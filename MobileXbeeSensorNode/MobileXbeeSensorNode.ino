/*****************************************************************
MobileXbeeSensorNode.ino 
Evan Chang-Siu
CSUM echangsiu@csum.edu
Original Creation Date: Aug 1, 2016
https://github.com/evancs/AllisionProject

MPU 9150 Code copied from:
Kris Winer for Sparkfun Electronics
Original Creation Date: April 8, 2014
https://github.com/sparkfun/MPU9150_Breakout


GPS code copied from from tinycircuits.com
https://tinycircuits.com/products/gps-tinyshield

This code utilizes a wireless Xbee as a node to send data from 
a MPU9150 IMU and tiny GPS.
Multiple Xbee's can be used and will be uniquely addressed when sent.

Development environment specifics:
	IDE: Arduino 1.7.9
	Hardware Platform: TinyDuino 3.3V/8MHz
	GPS Tiny Shield
        Tinycirucits mpu9150 tiny shield: https://tinycircuits.com/products/mpu9150-tinyshield

The MPU9150 is a versatile 9DOF sensor. It has a built-in
accelerometer, gyroscope, and magnetometer that
functions over I2C. It is very similar to the 6 DoF MPU6050 for which an extensive library has already been built.
Most of the function of the MPU9150 can utilize the MPU6050 library.

This Arduino sketch utilizes Jeff Rowberg's MPU6050 library to generate the basic sensor data
for use in two sensor fusion algorithms becoming increasingly popular with DIY quadcopter and robotics engineers.
I have added and slightly modified Jeff's library here.

* How to calculate actual acceleration, rotation speed, magnetic
  field strength using the  specified ranges as described in the data sheet:
  http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/IMU/PS-MPU-9150A.pdf
  
Tait-Bryan angles representing the sensor yaw, pitch, and roll angles suitable for any vehicle stablization control application.

Hardware setup: This library supports communicating with the
MPU9150 over I2C. These are the only connections that need to be made:
	MPU9150 --------- Arduino
	 SCL ---------- SCL (A5 on older 'Duinos')
	 SDA ---------- SDA (A4 on older 'Duinos')
	 VDD ------------- 3.3V
	 GND ------------- GND

Distributed as-is; no warranty is given.
*****************************************************************/
#include <SoftwareSerial.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"
#include "TinyGPS.h"

// Declare device MPU6050 class
MPU6050 mpu;

// The TinyGPS++ object
TinyGPS gps;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

const int GPS_ONOFFPin = A3;
const int GPS_SYSONPin = A2;
const int RXPin = A1, TXPin = A0;
const uint32_t GPSBaud = 9600;

int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3;     // raw data arrays reading
uint16_t count = 0;  // used to control display output rate
uint16_t delt_t = 0; // used to control display output rate
uint16_t mcount = 0; // used to control display output rate
uint8_t MagRate;     // read rate for magnetometer data

float pitch, yaw, roll;
float deltat = 0.0f;        // integration interval for both filter schemes
uint16_t lastUpdate = 0; // used to calculate integration interval
uint16_t now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mxraw, myraw, mzraw; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//Wireless data transfer
int datapacket[3];

void setup()
{
  Serial.begin(38400); // Start serial at 38400 bps

  // initialize MPU6050 device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU9150 connection successful") : F("MPU9150 connection failed"));

  // Set up the accelerometer, gyro, and magnetometer for data output
  mpu.setRate(7); // set gyro rate to 8 kHz/(1 * rate) shows 1 kHz, accelerometer ODR is fixed at 1 KHz

  MagRate = 10; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values

  // Digital low pass filter configuration.
  // It also determines the internal sampling rate used by the device as shown in the table below.
  // The accelerometer output rate is fixed at 1kHz. This means that for a Sample
  // Rate greater than 1kHz, the same accelerometer sample may be output to the
  // FIFO, DMP, and sensor registers more than once.
  /*
   *          |   ACCELEROMETER    |           GYROSCOPE
   * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
   * ---------+-----------+--------+-----------+--------+-------------
   * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
   * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
   * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
   * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
   * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
   * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
   * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
   */
  mpu.setDLPFMode(4); // set bandwidth of both gyro and accelerometer to ~20 Hz

  // Full-scale range of the gyro sensors:
  // 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
  mpu.setFullScaleGyroRange(0); // set gyro range to 250 degrees/sec

  // Full-scale accelerometer range.
  // The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
  mpu.setFullScaleAccelRange(0); // set accelerometer to 2 g range

  mpu.setIntDataReadyEnabled(true); // enable data ready interrupt

  ss.begin(GPSBaud);
  pinMode(GPS_SYSONPin, INPUT);
  digitalWrite(GPS_ONOFFPin, LOW);
  pinMode(GPS_ONOFFPin, OUTPUT);
  delay(100);
  Serial.print("Attempting to wake GPS module.. ");
  while (digitalRead( GPS_SYSONPin ) == LOW )
  {
    // Need to wake the module
    digitalWrite( GPS_ONOFFPin, HIGH );
    delay(5);
    digitalWrite( GPS_ONOFFPin, LOW );
    delay(100);
  }
  Serial.println("done.");

  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println();
  Serial.println("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum");
  Serial.println("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail");
  Serial.println("-------------------------------------------------------------------------------------------------------------------------------------");

}

void loop()
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  if (mpu.getIntDataReadyStatus() == 1) { // wait for data ready status register to update all data registers
    mcount++;
    // read the raw sensor data
    mpu.getAcceleration  ( &a1, &a2, &a3  );
    ax = a1 * 2.0f / 32768.0f; // 2 g full range for accelerometer
    ay = a2 * 2.0f / 32768.0f;
    az = a3 * 2.0f / 32768.0f;

    mpu.getRotation  ( &g1, &g2, &g3  );
    gx = g1 * 250.0f / 32768.0f; // 250 deg/s full range for gyroscope
    gy = g2 * 250.0f / 32768.0f;
    gz = g3 * 250.0f / 32768.0f;

    //  The gyros and accelerometers can in principle be calibrated in addition to any factory calibration but they are generally
    //  pretty accurate. You can check the accelerometer by making sure the reading is +1 g in the positive direction for each axis.
    //  The gyro should read zero for each axis when the sensor is at rest. Small or zero adjustment should be needed for these sensors.
    //  The magnetometer is a different thing. Most magnetometers will be sensitive to circuit currents, computers, and
    //  other both man-made and natural sources of magnetic field. The rough way to calibrate the magnetometer is to record
    //  the maximum and minimum readings (generally achieved at the North magnetic direction). The average of the sum divided by two
    //  should provide a pretty good calibration offset. Don't forget that for the MPU9150, the magnetometer x- and y-axes are switched
    //  compared to the gyro and accelerometer!

    //  if (mcount > 1000 / MagRate) { // this is a poor man's way of setting the magnetometer read rate (see below)
    mpu.getMag  ( &m1, &m2, &m3 );
    mxraw = m1 * 10.0f * 1229.0f / 4096.0f + 0.0f;//+ 18.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
    myraw = m2 * 10.0f * 1229.0f / 4096.0f + 0.0f;//+ 70.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
    mzraw = m3 * 10.0f * 1229.0f / 4096.0f + 0.0f;//+ 270.0f;

    mcount = 0;
    // }
  }

  now = micros();
  deltat = ((now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  myraw,  mxraw, -mzraw, 0);

  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  if (delt_t > 500) { // update LCD once per half-second independent of read rate

    //Print raw IMU data
    Serial.print("ax = "); Serial.print((int)1000 * ax);
    Serial.print(" ay = "); Serial.print((int)1000 * ay);
    Serial.print(" az = "); Serial.print((int)1000 * az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2);
    Serial.print(" gy = "); Serial.print( gy, 2);
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mxraw );
    Serial.print(" my = "); Serial.print( (int)myraw );
    Serial.print(" mz = "); Serial.print( (int)mzraw ); Serial.println(" mG");

    //Serial.print("q0 = "); Serial.print(q[0]);
    //Serial.print(" qx = "); Serial.print(q[1]);
    //Serial.print(" qy = "); Serial.print(q[2]);
    //Serial.print(" qz = "); Serial.println(q[3]);


    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI ; //- 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;

    //Print IMU data
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    //Serial.print("rate = "); Serial.print((float)1.0f / deltat, 2); Serial.println(" Hz");
    
    //Print GPS data
    print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
    print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
    gps.f_get_position(&flat, &flon, &age);
    print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
    print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
    print_date(gps);
    print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
    print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
    print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
    print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
    print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
    print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
    print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

    gps.stats(&chars, &sentences, &failed);
    print_int(chars, 0xFFFFFFFF, 6);
    print_int(sentences, 0xFFFFFFFF, 10);
    print_int(failed, 0xFFFFFFFF, 9);
    Serial.println();

    //Wireless Data Transmission *********************************************
    //load datapacket
    datapacket[0]=yaw;
    datapacket[1]=pitch;
    datapacket[2]=roll;    
   
    //send wireless data
    sendData(datapacket, sizeof(datapacket));

    count = millis();    
  }
}


// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool MagOn)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;

  if (MagOn == 1) {
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;
  } else {
    //Set to Zero to Remove Magnetometer
    _2bx = 0;
    _2bz = 0;
    _4bx = 0;
    _4bz = 0;
  }

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}


static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
            month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartdelay(0);
}

void sendData(int Data[], int datalength) {
  Serial.write(0x7E); //start byte

  //16 bytes  //length
  Serial.write((byte)0x00); //MSB
  
  Serial.write(0x0E + datalength); //LSB

  Serial.write(0x10); //Frame type
  Serial.write(0x01); //FrameID

  //RECEIVER
  Serial.write((byte)0x00); //64-bit dest. address
  Serial.write(0x13); //64-bit dest. address
  Serial.write(0xA2); //64-bit dest. address
  Serial.write((byte)0x00); //64-bit dest. address
  Serial.write(0x41); //64-bit dest. address
  Serial.write(0x52); //64-bit dest. address
  Serial.write(0x78); //64-bit dest. address
  Serial.write(0xB6); //64-bit dest. address
  
  //SENDER
  // Serial.write(0x00 ); //64-bit dest. address
  // Serial.write(0x13 ); //64-bit dest. address
  // Serial.write(0xA2 ); //64-bit dest. address
  // Serial.write(0x00 ); //64-bit dest. address
  // Serial.write(0x41 ); //64-bit dest. address
  // Serial.write(0x52 ); //64-bit dest. address
  // Serial.write(0x78 ); //64-bit dest. address
  // Serial.write(0xA0 ); //64-bit dest. address

  //Broadcast
  // Serial.write( (byte)0x0); //64-bit dest. address
  // Serial.write( (byte)0x0); //64-bit dest. address
  // Serial.write( (byte)0x0); //64-bit dest. address
  // Serial.write( (byte)0x0); //64-bit dest. address
  // Serial.write( (byte)0x0); //64-bit dest. address
  // Serial.write( (byte)0x0); //64-bit dest. address
  // Serial.write(0xFF); //64-bit dest. address
  // Serial.write(0xFF); //64-bit dest. address

  Serial.write(0xFF); //16-bit dest. address
  Serial.write(0xFE); //16-bit dest. address

  Serial.write((byte)0x00); //broadcast radius

  Serial.write(0xC0); //Options

  unsigned long chexsum = 0x10 + 0x01 + 0x13 + 0xA2 + 0x41 + 0x52 + 0x78 + 0xB6 + 0xFF + 0xFE + 0xC0; //Add up all previous bytes
  
  //Compute sum of data packet
  for (int i = 0; i<3; i++){
  Serial.write((byte)(Data[i]>>8)); //MSB
  Serial.write((byte)Data[i]); //LSB
  
  chexsum = chexsum+(byte)(Data[i]>>8)+(byte)Data[i]; //add all data packets
  } 

  Serial.write(0xFF - (chexsum & 0xFF)); //Checksum
}





