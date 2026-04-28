/**
 * Eduardo Diaz
 * PWM demo code with serial input
 * 
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
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

// uS per frame
#define FRAME_RATE 33000

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

// Variable to hold PWM slice number
uint slice_num ;

// PWM duty cycle
volatile int control;
volatile int old_control;

// Filtered accelerometer data
volatile fix15 filtered_accel_x = 0;
volatile fix15 filtered_accel_z = 0;
fix15 alpha = float2fix15(0.15); // Smoothing factor

fix15 complementary_angle = 0;

// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(slice_num);

    // Update duty cycle
    if (control!=old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    float raw_angleXZ = (atan2(fix2float15(acceleration[0]), fix2float15(acceleration[2])) * 180.0 / 3.14) + 90; // [0, 180] degree range
    // printf("raw_angleXZ = %f\n", raw_angleXZ);

    // Perform the accelerometer filtering
    // filtered_accel_x = multfix15(alpha, acceleration[0]) + multfix15((float2fix15(1.0) - alpha), filtered_accel_x);
    filtered_accel_x = filtered_accel_x + ((acceleration[0] - filtered_accel_x) >> 3);
    // filtered_accel_z = multfix15(alpha, acceleration[2]) + multfix15((float2fix15(1.0) - alpha), filtered_accel_z);
    filtered_accel_z = filtered_accel_z + ((acceleration[2] - filtered_accel_z) >> 3);

    float filtered_angleXZ = (atan2(fix2float15(filtered_accel_x), fix2float15(filtered_accel_z)) * 180.0 / 3.14) + 90; // [0, 180] degree range
    // printf("filtered_angleXZ = %f\n", filtered_angleXZ);

    // Estimate the arm angle with a complementary filter
    // No small angle approximation
    fix15 accel_angle = multfix15(float2fix15(atan2(fix2float15(filtered_accel_x), fix2float15(filtered_accel_z))), oneeightyoverpi) + int2fix15(90);
    // printf("accel_angle = %f\n", fix2float15(accel_angle));

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    fix15 gyro_angle_delta = multfix15(gyro[1], zeropt001);

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);
    // printf("complementary_angle = %f\n", fix2float15(complementary_angle));

    // printf("on_pwm_wrap()\n");
    // printf("acceleration = %f\n", fix2float15(acceleration[0]));

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

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    sprintf(screentext, "X-axis (WHITE), Y-axis (RED), Z-axis (GREEN)");
    setCursor(0, 0);
    writeString(screentext) ;

    // Draw bottom plot - accelerometer
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "Accelerometer");
    setCursor(0, 270);
    writeString(screentext);
    sprintf(screentext, "0") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+2") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "-2") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot - gyroscope
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "Gyroscope");
    setCursor(0, 50);
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+250") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-250") ;
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

            // Erase a column
            drawVLine(xcoord, 10, 480, BLACK) ;

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(filtered_accel_x)*120.0)-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[1])*120.0)-OldMin)/OldRange)), RED) ;
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(filtered_accel_z)*120.0)-OldMin)/OldRange)), GREEN) ;

            // Draw top plot
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[0]))-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[1]))-OldMin)/OldRange)), RED) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[2]))-OldMin)/OldRange)), GREEN) ;

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

// User input thread
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static int test_in ;
    static char num_in[16];
    static char c_in;
    static int result;
    static int num;
    while(1) {
        printf("Input a duty cycle (0-5000): ");
        fflush(stdout); // Ensure the prompt prints immediately

        if (fgets(num_in, sizeof(num_in), stdin) != NULL) {
            // Check if the user just hit Enter (empty input)
            if (num_in[0] == '\n') continue;

            test_in = atoi(num_in);
            printf("Duty cycle = %d\n", test_in);

            if (test_in >= 0 && test_in <= 5000) {
                control = test_in;
                // break; // Uncomment if you want to exit the loop after success
            } else {
                printf("Error: Out of range.\n");
            }
        }
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}


int main() {

    // Overclock
    // set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

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
    pt_schedule_start ;

}
