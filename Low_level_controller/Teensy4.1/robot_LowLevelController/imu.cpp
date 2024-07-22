#include "imu.h"


void IMU::imuInit(sensor_msgs__msg__Imu* imuMsg, sensor_msgs__msg__MagneticField* magMsg) {
  SPI_PORT.begin();
  myICM.begin(CS_PIN, SPI_PORT, SPI_FREQ);

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2;  // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                   // gpm2
                   // gpm4
                   // gpm8
                   // gpm16

  myFSS.g = dps250;  // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                     // dps250
                     // dps500
                     // dps1000
                     // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();

  this->imuMsg = imuMsg;
  this->magMsg = magMsg;
}

void IMU::updateData() {

  byte i;
  float temp[3], accVal[3], magVal[3];

  myICM.getAGMT();
  // apply gyro offsets (bias)

  imuMsg->angular_velocity.x = (double)(Gscale * (myICM.gyrX() - G_offset[0]));
  imuMsg->angular_velocity.y = (double)(Gscale * (myICM.gyrY() - G_offset[1]));
  imuMsg->angular_velocity.z = (double)(Gscale * (myICM.gyrZ() - G_offset[2]));

  accVal[0] = myICM.accX();
  accVal[1] = myICM.accY();
  accVal[2] = myICM.accZ();

  magVal[0] = myICM.magX();
  magVal[1] = myICM.magY();
  magVal[2] = myICM.magZ();

  // apply accel offsets (bias)

  for (i = 0; i < 3; i++)
    temp[i] = (accVal[i] - A_B[i]);
  imuMsg->linear_acceleration.x = (double)(Ascale * (A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2]));
  imuMsg->linear_acceleration.y = (double)(Ascale * (A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2]));
  imuMsg->linear_acceleration.z = (double)(Ascale * (A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2]));

  imuMsg->orientation = { 0 };

  imuMsg->orientation_covariance[0] = -1;
  imuMsg->linear_acceleration_covariance[0] = -1;
  imuMsg->angular_velocity_covariance[0] = -1;

  // apply mag offsets (bias)

  for (i = 0; i < 3; i++)
    temp[i] = (magVal[i] - M_B[i]);
  magMsg->magnetic_field.x = (double)(Mscale * (M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2]));
  magMsg->magnetic_field.y = (double)(Mscale * (M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2]));
  magMsg->magnetic_field.z = (double)(Mscale * (M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2]));
}
