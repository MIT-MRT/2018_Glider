#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>

/*  TO DO:
 *   - find out what angle the servo is at in max and min syringe displacement
 *   - 
 * 
 */

Servo myservo;
double accelX;
double servoAngle; // The angle of rotation from servos to syringes
bool shouldDive = false;
int g = 1;
int previousTime = 0; // used for taking the time in milliseconds 

LSM9DS1 imu;
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED 
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

void setup() {
 
  Serial.begin(115200);
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling, imu.begin(), which verifies communication with the IMU and turns it on
  
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }

  myservo.attach(2);
}

   
void loop() {
  
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  call readGyro()
    imu.readGyro();
  }

  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }

  // printGyro();
  // printAccel();

  // 
  accelX = imu.az;
  // Serial.println(accelX);
  // Mapping into range of 90-180 degrees, writing to servo for physical representation

  accelX = map(accelX,380,0,90,180);
  Serial.println(accelX);

  // map the syringe max and min in terms of rotation of the servos

  // myservo.write(accelX); // -----------------
  
  delay(15); // Give system time to react
  
   /* Control the buoyancy of the glider by attaching to syringes
    * - SERVO controls the syringes
    * 
    * As long as we are going down faster than we want to AND the angle was not set to max
    *  - increase angle to max (ensure that it is floating by maximize the air in syringes)
   */
  

  // Getting time since program started in milliseconds, to be a
  previousTime = millis();
  // Serial.println(previousTime);

  // NEED -> servoMin, servoMax
  /*
   *  c_1: some constant for servo bias, must be measured
   *  c_2: 90 or 180 depending on the servo
   *  c_3: smoothness parameter (does dive/rise when error in acceleration is 1 g or 0.1 g?)  
  */


  // changing the accel reading we want, according to some requirement of going up or down
  
  desiredAccel = someFunction()  //-----------

  // Desired change in g, to be shifted by c2 * tanh() to a number between 
  
  numbOfG = c_3 * ( accelX - desiredAccel ) //  -----

  // Setting desired angle change
  
  desiredAngleChange = c_1 +- c_2 * tanh(numbOfG); // -------
  
  // If we want to reduce the angle but the angle but servoAngle is less than or equal to the minimum set servo angle -> set angle to c1 AKA "do nothing"/
  if (desiredAngleChange < 0 && servoAngle <= servoMin)
  {
      servo.write( c_1 ); // or whatever zero means
  } 
  else if (desiredAngleChange > 0 && servoAngle >= servoMax) // if we want to increase angle but the angle is larger than or equal to max -> set angle to c1
  {
      servo.write( c_1 ); // or whatever zero means
  } 
  else // we want to change the angle in a direction that does not interfere with the restrictions
  {
      servoAngle += desiredAngleChange * (millis() - previousTime); // dividing by milliseconds ok?
      servo.write(servoAngle);
  }

}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}


void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}
