/**
 * Eduardo Diaz
 * 
 * PID control of 1-D Helicopter with serial input to adjust PID values and the desired angle of the beam.
 * Connecting to a VGA display allows for a visual look at the actual beam angle and 
 * the motor command signal.
 * 
 * Inspired by https://vanhunteradams.com/Pico/Helicopter/Helicopter.html
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 0 ---> VGA Hsync
 *  - GPIO 1 ---> VGA Vsync
 *  - GPIO 2 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 3 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 4 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 5 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 26 ---> MPU6050 SDA
 *  - GPIO 27 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 *  - GPIO 14 ---> PWM output
 * 
 */

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
// Include custom libraries
#include "vga16_graphics_v2.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[70];

// draw speed
int threshold = 15;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  25.0

// GPIO we're using for PWM
#define PWM_OUT 14

// GPIO ISR PIN
#define ISR_PIN 9

// Wait time for button debounce (currently mitigates noisy signal issue)
#define DEBOUNCE_US 30000


// Variable to hold PWM slice number
uint slice_num ;

// PWM duty cycle
volatile int old_duty_cycle;
volatile int duty_cycle;
volatile int filtered_duty_cycle = 0; // Goes out to VGA Display (Looks better filtered)

// Filtered accelerometer data
volatile fix15 filtered_accel_x = 0;
volatile fix15 filtered_accel_z = 0;

// PID variables
volatile uint desired_angle = 0;
volatile fix15 complementary_angle = 0;
volatile fix15 angle_error = 0;
volatile fix15 prev_angle_error = 0;
volatile fix15 integral = 0;
volatile float Kp = 165.0;
volatile float Ki = 45.6;
volatile float Kd = 41000.0;

volatile fix15 accel_angle = 0;
volatile fix15 gyro_angle_delta = 0;

volatile bool run_demo = false;

volatile int interrupt_count = 0;
volatile uint32_t last_irq_time = 0;


// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(slice_num);

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // Perform the accelerometer filtering
    filtered_accel_x = filtered_accel_x + ((acceleration[0] - filtered_accel_x) >> 5);
    filtered_accel_z = filtered_accel_z + ((acceleration[2] - filtered_accel_z) >> 5);

    // Estimate the arm angle with a complementary filter
    // No small angle approximation
    // Initial hanging down position set to 0 degrees
    accel_angle = multfix15(float2fix15(atan2(fix2float15(filtered_accel_x), fix2float15(filtered_accel_z))), oneeightyoverpi) + int2fix15(96);
    // printf("accel_angle = %f\n", fix2float15(accel_angle));

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    gyro_angle_delta = multfix15(gyro[1], zeropt001);

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);
    // printf("complementary_angle = %f\n", fix2float15(complementary_angle));

    // No error at rest
    if (desired_angle == 0) {
        angle_error = 0;
        integral = 0;
    } else {
        angle_error = int2fix15(desired_angle) - complementary_angle;
    }
    // printf("Angle error = %d\n", fix2int15(angle_error));

    // Proportional Term
    fix15 p_term = multfix15(float2fix15(Kp), angle_error);

    // Integral term
    integral += multfix15(angle_error, zeropt001);
    fix15 i_term = multfix15(float2fix15(Ki), integral);

    // Derivative term
    fix15 derivative = divfix((angle_error - prev_angle_error), zeropt001);

    fix15 d_term = multfix15(float2fix15(Kd), derivative);

    // Save error
    prev_angle_error = angle_error;

    // Total PID output
    duty_cycle = fix2int15(p_term + i_term + d_term);
    // Perform the duty_cycle filtering
    filtered_duty_cycle = filtered_duty_cycle + ((duty_cycle - filtered_duty_cycle) >> 8);

    if (duty_cycle < 0) {
        duty_cycle = 0;
    }

    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
    
    // printf("P Controller duty cycle = %d\n", duty_cycle);

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    static float old_range_angle = 130.0;
    static float new_range_angle = 150.0;

    static float old_range_pwm = 5000.0;
    static float new_range_pwm = 150.0;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    sprintf(screentext, "Current Beam Angle = ");
    setCursor(0, 0);
    writeString(screentext);

    sprintf(screentext, "Angle Error = ");
    setCursor(0, 9);
    writeString(screentext);

    // Draw bottom plot - Low-passed motor command signal
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "Motor Command");
    setCursor(0, 270);
    writeString(screentext);
    sprintf(screentext, "2500") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "5000") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot - Actual beam angle
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "Beam Angle");
    setCursor(0, 50);
    writeString(screentext) ;
    sprintf(screentext, "65°") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "130°") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "0°") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // drawRect(126, 0, 4, 3, RED);

            // Update current beam angle
            drawRect(126, 0, 36, 8, BLACK);
            fillRect(126, 0, 36, 8, BLACK);
            setCursor(126, 0);
            sprintf(screentext, "%.1f°", fix2float15(complementary_angle));
            writeString(screentext);

            // Update current angle error
            drawRect(90, 9, 36, 8, BLACK);
            fillRect(90, 9, 36, 8, BLACK);
            setCursor(90, 9);
            sprintf(screentext, "%.1f°", fix2float15(angle_error));
            writeString(screentext);

            // Erase a column
            drawVLine(xcoord, 10, 480, BLACK) ;

            // Draw bottom plot - Low-passed PWM motor command signal
            // drawPixel(xcoord, 430 - (int)(new_range_pwm*((float)((float)duty_cycle)/old_range_pwm)), GREEN) ;
            drawPixel(xcoord, 430 - (int)(new_range_pwm*((float)((float)filtered_duty_cycle)/old_range_pwm)), GREEN);


            // Draw top plot - Actual beam angle
            drawPixel(xcoord, 230 - (int)(new_range_angle*((float)(fix2float15(complementary_angle))/old_range_angle)), GREEN);


            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

void print_parameters() {
    printf("\nCurrent Parameters:\n");
    printf("Desired Angle = %d deg\n", desired_angle);
    printf("Kp = %.3f\n", Kp);
    printf("Ki = %.3f\n", Ki);
    printf("Kd = %.3f\n\n", Kd);
}

void ask_user_input() {
    printf("Select parameter to change:\n");
    printf("1 -> Desired Angle\n");
    printf("2 -> Kp\n");
    printf("3 -> Ki\n");
    printf("4 -> Kd\n");
    printf("5 -> Demo Sequence\n");
    printf("q -> Quit\n");
}

#define MAX_CMD_LEN 64
// ==================================================
// === User input thread
// ==================================================
static PT_THREAD(protothread_serial(struct pt *pt)) {
    PT_BEGIN(pt);

    static char input[MAX_CMD_LEN];
    static int index;
    static int c;

    // state variables
    static int menu_state;
    static int selected_param;

    index = 0;
    menu_state = 0;

    while (1) {

        // =========================
        // MAIN MENU
        // =========================
        if (menu_state == 0) {

            print_parameters();
            ask_user_input();

            menu_state = 1;
        }

        // =========================
        // GET USER INPUT
        // =========================
        c = getchar_timeout_us(0);

        if (c != PICO_ERROR_TIMEOUT) {

            // ENTER
            if (c == '\r' || c == '\n') {

                input[index] = '\0';

                printf("\n");

                // =========================
                // MENU COMMAND MODE
                // =========================
                if (menu_state == 1) {

                    if (!strcmp(input, "1")) {

                        selected_param = 1;

                        printf("Enter desired angle: ");

                        menu_state = 2;
                    } else if (!strcmp(input, "2")) {

                        selected_param = 2;

                        printf("Enter Kp: ");

                        menu_state = 2;
                    } else if (!strcmp(input, "3")) {

                        selected_param = 3;

                        printf("Enter Ki: ");

                        menu_state = 2;
                    } else if (!strcmp(input, "4")) {

                        selected_param = 4;

                        printf("Enter Kd: ");

                        menu_state = 2;
                    } else if (!strcmp(input, "5")) {

                        run_demo = true;

                        menu_state = 0;
                    } else if (!strcmp(input, "q")) {

                        printf("Quit selected\n");

                        menu_state = 0;
                    }

                    else {

                        printf("Invalid selection\n");

                        menu_state = 0;
                    }
                }

                // =========================
                // VALUE ENTRY MODE
                // =========================
                else if (menu_state == 2) {

                    switch (selected_param) {
                        case 1:
                            desired_angle = atoi(input);
                            break;
                        case 2:
                            Kp = atof(input);
                            break;
                        case 3:
                            Ki = atof(input);
                            break;
                        case 4:
                            Kd = atof(input);
                            break;
                    }

                    printf("Parameter updated!\n");

                    menu_state = 0;
                }

                // clear input buffer
                index = 0;
            }

            // BACKSPACE
            else if ((c == 8 || c == 127) && index > 0) {

                index--;

                printf("\b \b");
            }

            // NORMAL CHARACTER
            else if (index < MAX_CMD_LEN - 1) {

                input[index++] = c;

                putchar(c);
            }
        }

        PT_YIELD(pt);
    }

    PT_END(pt);
}

// ==================================================
// === GPIO Interrupt Thread
// ==================================================
static PT_THREAD(protothread_gpio(struct pt *pt)) {
    PT_BEGIN(pt);

    while (1) {

        if (run_demo) {

            run_demo = false;

            printf("\nInterrupt Triggered! (interrupt_count = %d)\n", interrupt_count);

            printf("Starting Demo!\n");

            printf("Horizontal!\n");
            desired_angle = 90;
            sleep_ms(5000);

            printf("30° Above Horizontal!\n");
            desired_angle = 120;
            sleep_ms(5000);

            printf("30° Below Horizontal!\n");
            desired_angle = 60;
            sleep_ms(5000);

            // Rest lever arm slowly
            desired_angle = 30;
            sleep_ms(2000);
            desired_angle = 15;
            sleep_ms(2000);
            desired_angle = 0;
            sleep_ms(2000);

            printf("Demo Finished!\n");

            print_parameters();
            ask_user_input();
        }

        PT_YIELD(pt);
    }

    PT_END(pt);
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

void gpio_callback() {

    uint32_t now = time_us_32();

    if (now - last_irq_time < DEBOUNCE_US) return;

    last_irq_time = now;

    interrupt_count++;
    run_demo = true;
}

int main() {

    // Overclock
    // set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA();

    gpio_init(ISR_PIN);
    gpio_set_dir(ISR_PIN, GPIO_IN);

    // Configure GPIO interrupt
    gpio_set_irq_enabled_with_callback(ISR_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's that they allocated to the PWM
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 7, same for 15)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    // pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_gpio);
    pt_schedule_start ;

}
