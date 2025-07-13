#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include <wiringPiI2C.h>

#define MPU_ADDRESS		0x68
#define SAMPLE_TIME_REG 	25 // GOR/1+DIV, GOR=8 when DLPF OFF, 1 when DLPF ON(see reg26)
#define CONFIG_REG		26 //Low pass filter

#define ACC_CONFIG_REG		28
#define GYRO_CONFIG_REG		27

#define TEMP_REG		65 //HIGH, 66: LOW
//------
#define ACC_X_REG		59
#define ACC_Y_REG		61
#define ACC_Z_REG		63
//------
#define GYRO_X_REG		67
#define GYRO_Y_REG		69
#define GYRO_Z_REG		71

#define INTERRUPT_REG		56
#define PWR_MANAGMENT_REG	107

//Power Management
typedef enum {
    MPU6050_CLK_INTERNAL_8MHZ   = 0, 
    MPU6050_CLK_PLL_XGYRO       = 1, 
    MPU6050_CLK_PLL_YGYRO       = 2, 
    MPU6050_CLK_PLL_ZGYRO       = 3, 
    MPU6050_CLK_PLL_EXT_32KHZ   = 4, 
    MPU6050_CLK_PLL_EXT_19MHZ   = 5, 
    MPU6050_CLK_RESERVED        = 6, 
    MPU6050_CLK_STOP            = 7 
} MPU6050_CLKSEL_t;

//CONFIG
typedef enum {
    MPU6050_DLPF_BW_260 = 0, 	//DLPF OFF
    MPU6050_DLPF_BW_184 = 1,
    MPU6050_DLPF_BW_94  = 2,
    MPU6050_DLPF_BW_44  = 3,
    MPU6050_DLPF_BW_21  = 4,
    MPU6050_DLPF_BW_10  = 5,
    MPU6050_DLPF_BW_5   = 6,
} MPU6050_DLPF_t;

typedef enum {
    MPU6050_EXT_SYNC_DISABLED = 0,
    MPU6050_EXT_SYNC_TEMP_OUT_L = 1,
    MPU6050_EXT_SYNC_GYRO_XOUT_L = 2,
    MPU6050_EXT_SYNC_GYRO_YOUT_L = 3,
    MPU6050_EXT_SYNC_GYRO_ZOUT_L = 4,
    MPU6050_EXT_SYNC_ACCEL_XOUT_L = 5,
    MPU6050_EXT_SYNC_ACCEL_YOUT_L = 6,
    MPU6050_EXT_SYNC_ACCEL_ZOUT_L = 7,
} MPU6050_ExtSync_t;

//Set Scale
typedef enum {
    MPU6050_GYRO_FS_250  = 0x00,	//+-250deg, 131.0
    MPU6050_GYRO_FS_500  = 0x08,	//+-500deg, 65.5
    MPU6050_GYRO_FS_1000 = 0x10,	//32.8
    MPU6050_GYRO_FS_2000 = 0x18		//16.4
} MPU6050_GYRO_CONFIG;

typedef enum {
    MPU6050_ACCEL_FS_2G  = 0x00,	//16384.0
    MPU6050_ACCEL_FS_4G  = 0x08,	//8192.0
    MPU6050_ACCEL_FS_8G  = 0x10,       	//4096.0
    MPU6050_ACCEL_FS_16G = 0x18		//2048.0
} MPU6050_ACCEL_CONFIG;


typedef struct{
    //Power Management
    uint8_t enable_temp;   // 1 = ON, 0 = OFF
    uint8_t sleep_mode;    // 1 = sleep, 0 = wake
    MPU6050_CLKSEL_t clock_sel;
    
    //Interrupt
    uint8_t data_ready_interrupt;  // interrupt when new data(bit 0)
    uint8_t fifo_overflow_interrupt; // interrupt when FIFO overflow (bit 6)
    uint8_t i2c_master_interrupt;  // interrupt when I2C master event (bit 5)
    
    //CONFIG
    MPU6050_ExtSync_t ext_sync;  // bits 5-3
    MPU6050_DLPF_t dlpf_cfg;     //bits 2-0
    
    //Set Scale
    MPU6050_ACCEL_CONFIG accel_config;
    MPU6050_GYRO_CONFIG gyro_config;
        
} MPU6050_CONFIG_t;

//---
int mpu6050_init(const MPU6050_CONFIG_t *config);
void mpu6050_set_sample_time(uint8_t samptime_div);
int16_t read_raw(uint8_t reg);

float mpu6050_read_accel(uint8_t reg);
float mpu6050_read_gyro(uint8_t reg);
void mpu6050_read_accelXYZ(float *accX, float *accY, float *accZ);
void mpu6050_read_gyroXYZ(float *gyroX, float *gyroY, float *gyroZ);

uint64_t millis();

#endif //MPU6050_H
