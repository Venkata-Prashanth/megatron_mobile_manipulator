#ifndef _CONFIG_H_
#define _CONFIG_H_

/* ROBOT 

    Front
MOTOR1  MOTOR2
    Back

*/

/* Encoder parameters*/
#define M1EncCh 1  // Quadencoder channel for motor1
#define M2EncCh 2  // Quadencoder channel for motor2

#define M1EncPinA 0  // encoder PhaseA Pin  for motor1
#define M1EncPinB 1  // encoder PhaseB Pin  for motor1

#define M2EncPinA 3  // encoder PhaseA Pin  for motor2
#define M2EncPinB 2  // encoder PhaseB Pin  for motor2

#define Encoder_Resolution 20480  //Encoder count resolution

/* Motorcontroller parameters*/
#define BAUDRATE 115200

/* Robot Kinematics */
#define Wheel_Radius 0.15  // wheel radius in meters
#define Track_Width 0.525  // distance between left wheel to right wheel
#define MaxRPM 150         // Max rpm for the wheels

/* PID gains */
#define k_p 0
#define k_d 0
#define k_i 0

/*IMU*/
#define SPI_PORT SPI      // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 5000000  // You can override the default SPI frequency
#define CS_PIN 10         // Which pin you connect CS to. Used only when "USE_SPI" is defined

/*Calibration data*/

// The offset and matrix are obtained calibtaion code from https://github.com/jremington/ICM_20948-AHRS.git

static const float Gscale = 0.01745329251994;  //scaling factor to change dps to rad/sec
static const float G_offset[3] = { -0.48901, 0.00945, 0.05528 };


static const float Ascale = 0.0098;  //scaling factor to change mg to m/sec²

// Accel scale
const float A_B[3] = { 29.72, -51.85, 69.96 };

static const float A_Ainv[3][3] = {
  { 0.96512, 0.03608, 0.01154 },
  { 0.03608, 0.91931, -0.00324 },
  { 0.01154, -0.00324, 1.03336 }
};

static const float Mscale = 0.001;

// Mag scale
static const float M_B[3] = { -19, 19.04, -5.58 };

static const float M_Ainv[3][3] = {
  { 32.80822, 0.64192, 0.9839 },
  { 0.64192, 33.34404, -1.40326 },
  { 0.9839, -1.40326, 33.56121 }
};
#endif