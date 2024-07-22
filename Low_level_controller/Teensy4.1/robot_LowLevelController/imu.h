#ifndef _IMU_H_
#define _IMU_H_

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "config.h"

#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

class IMU {
private:
  ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
  sensor_msgs__msg__Imu* imuMsg;
  sensor_msgs__msg__MagneticField* magMsg;
public:
  void imuInit(sensor_msgs__msg__Imu* imuMsg, sensor_msgs__msg__MagneticField* magMsg);
  void updateData();
};

#endif
