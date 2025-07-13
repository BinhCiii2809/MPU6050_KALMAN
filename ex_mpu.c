#include "mpu6050.h"
#include <stdio.h>
#include <unistd.h> 
#include "KALMAN_FILTER.h"

//Create kalman
Kalman_t KalmanX;
Kalman_t KalmanY;

float kalAngleX, kalAngleY;
float gyroXrate, gyroYrate;
//---
int fd;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroXangle, gyroYangle;

float roll, pitch, yaw;

uint64_t timer;
float dt;
int main(){   	
    	MPU6050_CONFIG_t config = {
  		 //Power Management    	
    		.enable_temp = 0,
        	.sleep_mode = 0,
        	.clock_sel = MPU6050_CLK_PLL_XGYRO,
        	
        	//Interrupt
        	.data_ready_interrupt = 1,
        	.fifo_overflow_interrupt = 0,
        	.i2c_master_interrupt = 0,
        	
        	//CONFIG
        	.ext_sync = MPU6050_EXT_SYNC_DISABLED,
	        .dlpf_cfg = MPU6050_DLPF_BW_94,
	        
	        //Set Scale
	        .accel_config = MPU6050_ACCEL_FS_2G,
        	.gyro_config = MPU6050_GYRO_FS_250       
    	};
    	
    	fd= mpu6050_init(&config);
    	if(fd < 0){
		printf("Connect to MPU failed\n");
		printf("Error address: %d\n",fd);
		return -1;
	}
	
	mpu6050_set_sample_time(7); //1000Hz
	
	printf("Connect to MPU6050 successfully\n");
	usleep(100000); //100ms
	
	//Set starting angle
	mpu6050_read_accelXYZ(&accX, &accY, &accZ);
	
	//Calculate angle
	roll  = atan2f(accY, sqrtf(accX * accX + accZ * accZ)) * 180/M_PI; //rad to deg
    	pitch = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * 180/M_PI;
	
	//Set starting angle kalman
	kalman_setAngle(&KalmanX, roll);
	kalman_setAngle(&KalmanY, pitch);
	
	gyroXangle= roll;
	gyroYangle= pitch;
	
	timer= millis();
    	while(1){
    		mpu6050_read_accelXYZ(&accX, &accY, &accZ);
    		mpu6050_read_gyroXYZ(&gyroX, &gyroY, &gyroZ);
   		
   		dt= (float)(millis()-timer) / 1000.0f;
   		timer= millis();
   		
   		//Calculate angle
		roll  = atan2f(accY, sqrtf(accX * accX + accZ * accZ)) * 180/M_PI; //rad to deg
    		pitch = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * 180/M_PI;
    		
    		//----
    		gyroXrate= gyroX;
    		gyroYrate= gyroY;
    		
    		//Fixes problem when accAngle between -180 deg to 180 deg
    		if((roll > 90 && kalAngleX < -90) || (roll < -90 && kalAngleX > 90)){
    			kalman_setAngle(&KalmanX, roll);
    			kalAngleX= roll;
    			gyroXangle= roll;
    		} else
    			kalAngleX= kalman_getAngle(&KalmanX, roll, gyroXrate, dt);
    		
    		//Invert the Y axis
    		if ((kalAngleX > 90) || (kalAngleX < -90))
  			gyroYrate = -gyroYrate;
  		kalAngleY = kalman_getAngle(&KalmanY, pitch, gyroYrate, dt);
  
  		//Fixes problem when accAngle between -180 deg to 180 deg
    		if((pitch > 90 && kalAngleY < -90) || (pitch < -90 && kalAngleY > 90)){
    			kalman_setAngle(&KalmanY, pitch);
    			kalAngleY= pitch;
    			gyroYangle= pitch;
    		} else
    			kalAngleY= kalman_getAngle(&KalmanY, pitch, gyroYrate, dt);
    		
    		if ((kalAngleY > 90) || (kalAngleY < -90))
  			gyroXrate = -gyroXrate;
  		kalAngleX = kalman_getAngle(&KalmanX, roll, gyroXrate, dt);
  		
  		// Calculate without any filter
  		gyroXangle += gyroXrate * dt;
 		gyroYangle += gyroYrate * dt;
 		
 		// Reset when singularity
  		if (gyroXangle < -180 || gyroXangle > 180)
   			gyroXangle = kalAngleX;
  		if (gyroYangle < -180 || gyroYangle > 180)
   			gyroYangle = kalAngleY;
   		
   		//Performance evaluation
   		printf("roll: %.2f\t",roll);
   		printf("gyroXangle: %.2f\t",gyroXangle);
   		printf("kalAngleX: %.2f\n",kalAngleX);
   		
   		printf("pitch: %.2f\t",pitch);
   		printf("gyroYangle: %.2f\t",gyroYangle);
   		printf("kalAngleY: %.2f\n",kalAngleY);
   		
   		usleep(1000); //1000Hz
    	}
	return 0;
}
