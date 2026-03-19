#ifndef __BSP_ICM20948_H
#define __BSP_ICM20948_H

#include <stdint.h>

typedef struct{ //中间私有变量
	float x;
	float y;
	float z;
}PrivateBuf_t;
	
typedef struct{
	PrivateBuf_t gyro; //3轴角速度
	PrivateBuf_t accel;//3轴加速度
	PrivateBuf_t magn;//3轴磁力计
}IMU_DATA_t;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}ATTITUDE_DATA_t;


//IMU操作接口
typedef struct {
    uint8_t (*Init)(void);   //返回值：1错误 0无异常
    uint8_t (*DeInit)(void); //返回值：1错误 0无异常
	
	void (*UpdateZeroPoint_axis)(void);               //更新9轴数据零点
	void (*UpdateZeroPoint_attitude)(void);          //更新姿态零点
    void (*Update_9axisVal)(IMU_DATA_t* imudata,IMU_DATA_t* ori);                         //更新9轴数据
	void (*UpdateAttitude)(IMU_DATA_t imudata,ATTITUDE_DATA_t *attitude); //更新姿态角
	
}IMUInterface_t,*pIMUInterface_t;

extern IMUInterface_t UserICM20948;

//用于存放IMU数据
extern IMU_DATA_t axis_9Val;      
extern IMU_DATA_t axis_9ValOri;  
extern ATTITUDE_DATA_t AttitudeVal;

#endif /* __BSP_IMU_H */
