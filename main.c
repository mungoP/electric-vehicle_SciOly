#include <hardware/gpio.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/pwm.h> 
#include <pico/stdio.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdint.h>
#include <stdio.h> 
#include "pico/stdlib.h" 
#include "hardware/i2c.h"
#include "pins.h"

#define WINDOW_SIZE 8
#define RAD_TO_DEG 57.2957795f

typedef struct {
    int16_t buffer[WINDOW_SIZE]; 
    uint8_t index; 
    uint32_t sum; 
} MovingAverage; 

// Global offset variables
int16_t gyro_bias[3] = {0, 0, 0};
int16_t accel_bias[3] = {0, 0, 0};

// Function prototypes
int16_t update_moving_average(MovingAverage *ma, int16_t new_value);
void init_motors();
void drive_fwd_test();
void stop_motors();
void init_mpu6050();
void read_mpu6050(int16_t accel[3], int16_t gyro[3]);
void calibrate_mpu6050();
float read_mt6701_angle(i2c_inst_t *i2c, uint8_t addr);


int main() {
    stdio_init_all(); 
    sleep_ms(2000); 
    printf("Starting EV Firmware Initialization...\n"); 

    // init moving average structs for gyro 
    MovingAverage gyro_x_filter = {0}; 
    MovingAverage gyro_y_filter = {0}; 
    MovingAverage gyro_z_filter = {0}; 

    // acceleration and gyroscope values
    int16_t accel[3], gyro[3]; 
    
    // init LED "heartbeat"
    gpio_init(LED_PIN); 
    gpio_set_dir(LED_PIN, GPIO_OUT); 
    
    gpio_init(START_BTN_PIN); 
    gpio_set_dir(START_BTN_PIN, GPIO_IN); 
    gpio_pull_up(START_BTN_PIN);

    // init_motors(); 

    // init I2C bus 0 - left encoder + IMU
    i2c_init(i2c0, 400 * 1000); // 400kHz fast mode zoom
    gpio_set_function(ENC_LEFT_SDA_PIN, GPIO_FUNC_I2C); 
    gpio_set_function(ENC_LEFT_SCL_PIN, GPIO_FUNC_I2C); 
    gpio_pull_up(ENC_LEFT_SDA_PIN); 
    gpio_pull_up(ENC_LEFT_SCL_PIN); 
    
    // init I2C bus 1 - right encoder
    i2c_init(i2c1, 400 * 1000); 
    gpio_set_function(ENC_RIGHT_SDA_PIN, GPIO_FUNC_I2C); 
    gpio_set_function(ENC_RIGHT_SCL_PIN, GPIO_FUNC_I2C); 
    gpio_pull_up(ENC_RIGHT_SDA_PIN); 
    gpio_pull_up(ENC_RIGHT_SCL_PIN); 

    // init IMU - mpu6050
    init_mpu6050(); 
    printf("Calibrating MPU6050... Keep the robot still.\n"); 
    calibrate_mpu6050();

    absolute_time_t last_time = get_absolute_time(); 

    while (true) {
        gpio_put(LED_PIN, 1); 

        absolute_time_t now = get_absolute_time(); 
        float dt = (float)absolute_time_diff_us(last_time, now) / 1000000.0f; 
        last_time = now; 
        if (dt <= 0.0f) dt = 0.01f; // fallback saftey

        read_mpu6050(accel, gyro); 

        int16_t smooth_gx = update_moving_average(&gyro_x_filter, gyro[0]); 
        int16_t smooth_gy = update_moving_average(&gyro_y_filter, gyro[1]); 
        int16_t smooth_gz = update_moving_average(&gyro_z_filter, gyro[2]); 

        float left_angle = read_mt6701_angle(i2c0, ENC_LEFT_ADDR); 
        float right_angle = read_mt6701_angle(i2c1, ENC_RIGHT_ADDR); 

        printf("IMU [Ax:%6d Ay:%6d Az:%6d | Gx:%6d Gy:%6d Gz:%6d] | Enc left: %6.2f | Enc right: %6.2f\n", 
                accel[0], accel[1], accel[2], 
                gyro[0], gyro[1], gyro[2],
                left_angle, right_angle);
        gpio_put(LED_PIN, 0); 
        sleep_ms(100); 
    }

    return 0; 
}

int16_t update_moving_average(MovingAverage *ma, int16_t new_value) {
    ma->sum -= ma->buffer[ma->index]; 
    ma->buffer[ma->index] = new_value; 
    ma->sum += new_value; 
    ma->index = (ma->index + 1) % WINDOW_SIZE; 
    return (int16_t)(ma->sum / WINDOW_SIZE); 
}

void init_motors() {
    gpio_init(IN1_PIN); 
    gpio_init(IN2_PIN); 
    gpio_init(IN3_PIN); 
    gpio_init(IN4_PIN); 

    gpio_set_dir(IN1_PIN, GPIO_OUT); 
    gpio_set_dir(IN2_PIN, GPIO_OUT); 
    gpio_set_dir(IN3_PIN, GPIO_OUT); 
    gpio_set_dir(IN4_PIN, GPIO_OUT); 
}

void drive_fwd_test() {
    gpio_put(IN1_PIN, 1); 
    gpio_put(IN2_PIN, 0); 
    gpio_put(IN3_PIN, 1); 
    gpio_put(IN4_PIN, 0); 
}

void stop_motors() {
    gpio_put(IN1_PIN, 0); 
    gpio_put(IN2_PIN, 0); 
    gpio_put(IN3_PIN, 0); 
    gpio_put(IN4_PIN, 0); 
}

void init_mpu6050() {
    uint8_t buf[2]; 
    buf[0] = 0x6b; 
    buf[1] = 0x00; 
    i2c_write_blocking(i2c0, IMU_ADDR, buf, 2, false); 
}

void read_mpu6050(int16_t accel[3], int16_t gyro[3]) {
    uint8_t reg = 0x3B; // first accelerometer register
    uint8_t buffer[14]; 

    i2c_write_blocking(i2c0, IMU_ADDR, &reg, 1, true); 
    i2c_read_blocking(i2c0, IMU_ADDR, buffer, 14, false); 

    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]); 
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]); 
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]); 
    
    gyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]); 
    gyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]); 
    gyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]); 

    gyro[0] -= gyro_bias[0]; 
    gyro[1] -= gyro_bias[1]; 
    gyro[2] -= gyro_bias[2]; 
}

void calibrate_mpu6050() {
    printf("Calibrating MPU6050... Keep the robot completely still!\n");
    int32_t gyro_sum[3] = {0, 0, 0};
    int32_t accel_sum[3] = {0, 0, 0};
    int num_samples = 200;

    for (int i = 0; i < num_samples; i++) {
        int16_t temp_accel[3], temp_gyro[3];
        read_mpu6050(temp_accel, temp_gyro);
        
        gyro_sum[0] += temp_gyro[0];
        gyro_sum[1] += temp_gyro[1];
        gyro_sum[2] += temp_gyro[2];
        
        accel_sum[0] += temp_accel[0];
        accel_sum[1] += temp_accel[1];
        accel_sum[2] += temp_accel[2]; // Note: Z-axis accel will naturally hover around 1g (~16384)
        
        sleep_ms(10);
    }

    gyro_bias[0] = gyro_sum[0] / num_samples;
    gyro_bias[1] = gyro_sum[1] / num_samples;
    gyro_bias[2] = gyro_sum[2] / num_samples;

    printf("Calibration complete! Gyro biases locked.\n");
}

float read_mt6701_angle(i2c_inst_t *i2c, uint8_t addr) {
    uint8_t reg = 0x03; // high byte register for angle
    uint8_t buffer[2]; 

    int ret1 = i2c_write_blocking(i2c, addr, &reg, 1, true); 
    int ret2 = i2c_read_blocking(i2c, addr, buffer, 2, false); 

    if (ret1 < 0 || ret2 < 0) {
        return -1.0f; // error if read faisl 
    }

    uint16_t raw_angle = ((buffer[0] << 6) | (buffer[1] >>2));
    float degrees = ((float)raw_angle / 16384.0f) * 360.0f; 
    return degrees; 

}
