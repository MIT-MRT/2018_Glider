#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>

// This defines parameters used globally for the servo adjustments.
#define PUSH 0      // Servo should push water out of the syringes
#define PULL 1      // Servo should pull water into the syringes
#define PUSHREST 2  // Servo waits after pushing
#define PULLREST 3  // Servo waits after pulling

// REMEMBER TO RESET POSITION ZERO ON THE POTENSIOMETER OF THE SERVO!
// THE HANDS ON WAY TO DO IT IS TO WRITE 90 TO THE SERVO, AND ADJUST THE POTMETER UNTIL SERVO STOPS

Servo myservo;
float timeSinceStart = 0; // used for taking the time in milliseconds by millis()

// variable to store the value coming from the sensor
int sensorValue = 0;  

// select the input pin for the potentiometer -> depends on which analog input 
int sensorPin = A0;    
int sensorThreshold = 0;

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
  myservo.attach(9);
}

   
void loop() {

  // Update the sensor values whenever new data is available
  if (imu.gyroAvailable()) { imu.readGyro(); }
  
  // Read from the accelerometer. Updates the ax, ay, and az variables.
  if ( imu.accelAvailable() ) { imu.readAccel(); }

  // printGyro(); printAccel(); // Used for printing the accelerometer values if needed

  sensorValue = analogRead(sensorPin); // reads sensor value from analog input
  Serial.println(sensorValue);
  sensorThreshold = 650; // measured with 5V power supply 
 
  timeSinceStart = millis() / 1000.0;  // Get time since the start of the program in seconds

  // moveTime simulates the time it takes to dive and rise for the glider (while servo is not moving) in milliseconds
  int moveTime = 2 * 1000;

  // Write to servo depending on rising or diving or doing nothing, see implementation below
  
  diveAndRise(&myservo, moveTime, sensorValue, sensorThreshold);

  delay(20); // do not remove
}

/////// OWN IMPLEMENTATIONS

// Determines the behaviour of the glider
void diveAndRise(Servo* myservo, int moveTime, int sensorValue, int sensorThreshold){

  // Setting the value of different rotations. Cannot control the speed (), only direction
  int rotateCounterClockwise = 45, rotateClockwise = 130, rotateNone = 90;

  // This parameter is setting the initial state of the SERVO! Not to be confused with the glider's state
  static int state = PUSH; // static so that it is remembered through every loop
  
  switch(state){
      case PUSH: // Push syringes -> empty for water
          if (sensorValue < sensorThreshold)
          {
              myservo->write(rotateClockwise);        // If the glider is supposed to dive -> fill the syringes
              Serial.println("filling syringes");     // Just for display
              delay(1000);                            // Used to avoid measuring spikes in sensorValue after setting rotation
          } else {
              state = PUSHREST;                       // sensorValue has surpassed the threshold -> syringes are empty
          }
          break;
      
      case PULL: // Pull sytinges -> fill them with water
          if (sensorValue < sensorThreshold){
              myservo->write(rotateCounterClockwise); // If the glider is supposed to dive -> fill the syringes
              Serial.println("filling syringes");     // Just for display
              delay(1000);                            // Used to avoid measuring spikes in sensorValue after setting rotation
          } else {
              state = PULLREST;                       // sensorValue has surpassed the threshold -> syringes are full
          }
          break;
      
      case PUSHREST: // Stopping servo after it has detected syringes being pushed into max position
          Serial.println("emptying syringes");
          myservo->write(rotateNone);
          delay(moveTime);
          state = PULL;
          break;

      case PULLREST: // Stopping servo after it has detected syringes being pulled into min position
          Serial.println("emptying syringes");
          myservo->write(rotateNone);
          delay(moveTime);
          state = PUSH;
          break;
  }
  
}


////// FROM ARDUINO EXAMPLE CODE

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
