#include "mpu6050.h"
#include "KALMAN_FILTER.h"

static int fd = -1;
static float accel_scale_factor = 10.0;
static float gyro_scale_factor = 10.0;   

typedef struct {
    uint8_t CLKSEL    : 3;
    uint8_t TEMP_DIS  : 1;
    uint8_t reserved1 : 1;	
    uint8_t CYCLE     : 1;
    uint8_t SLEEP     : 1;
    uint8_t reserved2 : 1;
} MPU6050_PWR_MGMT1_t;
	
int mpu6050_init(const MPU6050_CONFIG_t *config){
    fd= wiringPiI2CSetup(MPU_ADDRESS);
    if(fd < 0) return -1;
    
    //Power Management
    MPU6050_PWR_MGMT1_t reg = {0};

    reg.CLKSEL = config->clock_sel & 0x07; //0x07 -> 3bit
    reg.TEMP_DIS = config->enable_temp ? 0 : 1;  // Enable: 0, Disable: 1
    reg.SLEEP = config->sleep_mode;

    uint8_t pwr = *((uint8_t*)&reg);
    wiringPiI2CWriteReg8(fd, PWR_MANAGMENT_REG, pwr);
    
    //Interrupt
    uint8_t interrupt = 0;

    if (config->data_ready_interrupt)
        interrupt |= (1 << 0);  // DATA_RDY_EN (bit 0)

    if (config->i2c_master_interrupt)
        interrupt |= (1 << 5);  // I2C_MST_INT_EN (bit 5)

    if (config->fifo_overflow_interrupt)
        interrupt |= (1 << 6);  // FIFO_OFLOW_EN (bit 6)

    wiringPiI2CWriteReg8(fd, INTERRUPT_REG, interrupt);  // 0x38 = INT_ENABLE register
    
    //CONFIG
    uint8_t cfg = 0;
    
    cfg |= (config->ext_sync & 0x07) << 3;  // bits 5-3
    cfg |= (config->dlpf_cfg & 0x07);       // bits 2-0
    
    wiringPiI2CWriteReg8(fd, CONFIG_REG, cfg);
    
    //Set Scale
    wiringPiI2CWriteReg8(fd, GYRO_CONFIG_REG, config->gyro_config);
    wiringPiI2CWriteReg8(fd, ACC_CONFIG_REG, config->accel_config);
  
    switch (config->accel_config) {
        case MPU6050_ACCEL_FS_2G:  accel_scale_factor = 16384.0; break;
        case MPU6050_ACCEL_FS_4G:  accel_scale_factor = 8192.0; break;
        case MPU6050_ACCEL_FS_8G:  accel_scale_factor = 4096.0; break;
        case MPU6050_ACCEL_FS_16G: accel_scale_factor = 2048.0; break;
    }

    switch (config->gyro_config) {
        case MPU6050_GYRO_FS_250:  gyro_scale_factor = 131.0; break;
        case MPU6050_GYRO_FS_500:  gyro_scale_factor = 65.5; break;
        case MPU6050_GYRO_FS_1000: gyro_scale_factor = 32.8; break;
        case MPU6050_GYRO_FS_2000: gyro_scale_factor = 16.4; break;
    }
    
    return fd;
}

void mpu6050_set_sample_time(uint8_t samptime_div) { 			//set sample_time
    wiringPiI2CWriteReg8(fd, SAMPLE_TIME_REG, samptime_div);
}

int16_t read_raw(uint8_t reg) {				//read sensor MPU
    int16_t high = wiringPiI2CReadReg8(fd, reg);
    int16_t low  = wiringPiI2CReadReg8(fd, reg + 1);
    return (high << 8) | low;
}

float mpu6050_read_accel(uint8_t reg){
    return (float)read_raw(reg) / accel_scale_factor;
}

float mpu6050_read_gyro(uint8_t reg){
    return (float)read_raw(reg) / gyro_scale_factor;
}

void mpu6050_read_accelXYZ(float *accX, float *accY, float *accZ){
    *accX = mpu6050_read_accel(ACC_X_REG);
    *accY = mpu6050_read_accel(ACC_Y_REG);
    *accZ = mpu6050_read_accel(ACC_Z_REG);
}

void mpu6050_read_gyroXYZ(float *gyroX, float *gyroY, float *gyroZ){
    *gyroX = mpu6050_read_gyro(GYRO_X_REG); // deg/s
    *gyroY = mpu6050_read_gyro(GYRO_Y_REG);
    *gyroZ = mpu6050_read_gyro(GYRO_Z_REG);
}

uint64_t millis() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}
