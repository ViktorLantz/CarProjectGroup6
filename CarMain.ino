#include <Adafruit_NeoPixel.h>
#include <SonarSRF02.h>
#include <SonarSRF08.h>
#include <SonarSRF.h>
#include <Servo.h>
#include <Wire.h>

/*
      Car specifics:
      Locker code: 835
      Code to room: 9010


      Servo Neutral = 1105
      Connected to PIN 11
      ++ = turn Left

      Motor Neutral = 1500
      Connected to PIN 10
      ++ = drive Forward

      Interrupt sequence is connected to PIN 3

      LEFT TO RIGHT FROM IRSENSOR SIDE

      Servo: Brown-Red-Orange (DPIN 11)
      ESC: Blue-Nothing-Green (DPIN 10)
      Receiver: Grey-Purple-Blue (DPIN3)  -- SERVO
      Receiver: Nothing-Nothing-Green (DPIN5) -- MOTOR
      US Sensor Front pins: Green-Grey-Purple-Blue (I2C Bus) --- Main-Register: 0xE0
      US Sensor Front Right pins: Blue-Orange-Yellow-Green (I2C Bus) --- Main-Register: 0xE6
      IR Sensor Front Right Pin: Yellow-Red-Black (A0)
      IR Sensor Back Right Pin: Yellow-Red-Black (A1)
      IR Sensor Back Pin: Yellow-Red-Black (A2)
      LED Sensors: Purple-Grey-White (DPIN6)
      Wheel Encoder: Brown-Red-Orange-Yellow (DPIN12 & DPIN13)
      Voltage Regulator (Additional volts for the arduino board): Blue-Purple(SPI(VCC & GND))

*/

// Create servo object to control a servo
Servo myServo;
// Create servo object to control the motor
Servo myMotor;
// Create Sonar objects
SonarSRF08 USFront;
SonarSRF08 USFrontRight;


// Create LED lights object
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
// NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs) -- IF needed, this one should be used.
// NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
// NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products) -- IF needed, this one should be used.
// NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, 6);


int ch1, ch2;
volatile int flag;
int counter;
int midPoint;
double speedVal;
double steeringVal;
int defaultSteering = 1500;
int adjustedSteering = 1105;
int velocity = 0;
int steeringCounter = 0;


// A string to hold incoming data
String inputString = "";
String outputString = "";

// Setting IRSensor Analog Pins
int IRSensorFrontRight = 0;
int IRSensorBackRight = 1;
int IRSensorBack = 2;

// Printing purposes
int IRFrontRight = 0;
int IRBackRight = 0;
int IRBack = 0;

// Converted IR Value
int convertedIRFrontRight;
int convertedIRBackRight;
int convertedIRBack;

// Delimiter Strings
String us = "[u";
String ir = "[i";

// Main register - The USsensor address.
#define MAIN_08_ADDRESS (0xE0 >> 1) //E0
#define MAIN_08_ADDRESS2 (0xE6 >> 1) //E6
#define GAIN_REGISTER 0x09
#define LOCATION_REGISTER 0x8C

// i for inches, c for centimeters, m for micro_seconds
char unit = 'c';

void setup() {
  
  // initialize serial:
  Serial.begin(9600);
  
  // attaches the servo on pin 11. Max & Min boundaries
  myServo.attach(11, 700, 1700);
  // Sets the steering to neutral (Straight)
  myServo.writeMicroseconds(1105);
  // Attach the motor to pin 10. Max & Min boundaries
  myMotor.attach(10, 1000, 2000);
  // Sets the motor to neutral (Standstill)
  myMotor.writeMicroseconds(1500);

  // Set PIN 3 (Servo)
  // Set PIN 5 (Motor)
  pinMode(3, INPUT);
  pinMode(5, INPUT);

  // Listen for transfer of data from the encoder
  Wire.begin();

  // USSensors connection and register address location
  USFront.connect(MAIN_08_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER); 
  USFrontRight.connect(MAIN_08_ADDRESS2, GAIN_REGISTER, LOCATION_REGISTER);


  // LED strips, back and front, sets initial lighting to be off.
  strip.begin();
  strip.show();

  // Attach the interupt to pin 3 and run function stopALL if the value rises
  // Value will rise if the controller is switched on.
  attachInterrupt(digitalPinToInterrupt(3), stopAll, RISING);
}

void loop() {

  // Check if the flag has been set. (Flag is set if a pulse is received from pin 3)((The controller is switched on))
  // Set the car stats to Neutral
  // If the pulse from the receiver is sending 0s, this means that the remote controller is turned off.
  // Check five times to make sure that the controller is off, then return functionality (Set flag back to 0) to the serial.

  if (flag == 1) {

    // Get the values from US sensor front and front right, call distance function to print
    // Out the actual value from the sensors.
    
    int frontSensorReading = USFront.getRange(unit);
    //Serial.print(sensorReading);
    
    int frontRightSensorReading = USFrontRight.getRange(unit);
    //Serial.print(sensorReading);

    // IR sensor values. Low == close && less than ~8 is too close to the sensors so it bugsout.
    IRFrontRight = analogRead(IRSensorFrontRight);
    convertedIRFrontRight = IRConversion(IRFrontRight);
    //Serial.println(convertedIRFrontRight);
    
    IRBackRight = analogRead(IRSensorBackRight);
    convertedIRBackRight = IRConversion(IRBackRight);
    //Serial.println(convertedIRBackRight);
    
    IRBack = analogRead(IRSensorBack);
    convertedIRBack = IRConversion(IRBack);
    //Serial.println(convertedIRBack);

    outputString = us + frontSensorReading + ',' + frontRightSensorReading + ']' + ir + convertedIRFrontRight + ',' + convertedIRBack + ',' + convertedIRBackRight + ']';
    //Serial.print(outputString);

    // ch1 = servo, ch2 = motor
    ch1 = pulseIn(3, HIGH);
    ch2 = pulseIn(5, HIGH);

    // Request one byte of data from the encoder
    Wire.requestFrom(8, 1);
    if(Wire.available()) {
        velocity = Wire.read();    
     }

    Serial.println(velocity);   

    myServo.writeMicroseconds(1105);
    myMotor.writeMicroseconds(1500);

    // If ch1 decreases turn left by increasing the value of alteredSteering to approx 1400
    if (ch1 < 1450 && ch1 > 500) {
      int toSteer = defaultSteering - ch1;
      adjustedSteering = adjustedSteering + toSteer + 100;
      Serial.print("Value of adjustedSteering: ");
      Serial.println(adjustedSteering);
      Serial.print("Value of toSteer: ");
      Serial.println(toSteer);
      myServo.writeMicroseconds(adjustedSteering);
      adjustedSteering = 1105;
      
    }
    // If ch1 increases turn right by decreasing the value of alteredSteering to approx 800
    if (ch1 > 1560) {
      int toSteer = ch1 - defaultSteering;
      adjustedSteering = adjustedSteering - toSteer -100;
      Serial.print("Value of adjustedSteering: ");
      Serial.println(adjustedSteering);
      Serial.print("Value of toSteer: ");
      Serial.println(toSteer);
      myServo.writeMicroseconds(adjustedSteering);
      adjustedSteering = 1105;
    }
    // Drive Forward
    if (ch2 > 1570) {
      
      if(velocity < 2){
        //1570
        myMotor.writeMicroseconds(1570);
      }else{
        //1560
        myMotor.writeMicroseconds(1560);
      }
    }
    // Drive Backward
    if (ch2 < 1430 && ch2 > 500) {
      if(velocity < 2){
        myMotor.writeMicroseconds(1200);
        
      }else{
        myMotor.writeMicroseconds(1250);
      }
    }

    // Motor and Servo microsecond values errorTesting prints.
    //Serial.print("This is channel 1:");
    //Serial.println(ch1);
    //Serial.print("This is channel 2:");
    //Serial.println(ch2);


    // Ensure that the receiver is turned off by checking that the value of
    // ch1 == 0 for 5 runs of the loop.
    // Some testing of the average midpoint value of the servo (1105 for current knob settings)
    if (ch1 == 0) {
      counter++;

      //midPoint += ch1;
    } else {
      counter = 0;
    }

    if (counter > 5) {
      flag = 0;

      //midPoint = midPoint/10;
      //Serial.print("The middle point of 10 values:");
      //Serial.println(midPoint);
      //counter = 0;
      //midPoint = 0;
    }
  } else {

    // Get the values from US sensor front and front right, call distance function to print
    // Out the actual value from the sensors.
    
    int frontSensorReading = USFront.getRange(unit);
    //Serial.print(frontSensorReading);
    int frontRightSensorReading = USFrontRight.getRange(unit);
    //Serial.print(frontRightSensorReading);

    // IR sensor values. Low == close && less than ~8 is too close to the sensors so it bugsout.
    IRFrontRight = analogRead(IRSensorFrontRight);
    convertedIRFrontRight = IRConversion(IRFrontRight);
   
    IRBackRight = analogRead(IRSensorBackRight);
    convertedIRBackRight = IRConversion(IRBackRight);
    
    IRBack = analogRead(IRSensorBack);
    convertedIRBack = IRConversion(IRBack);

    
    
    outputString = us + frontSensorReading + ',' + frontRightSensorReading + ']' + ir + convertedIRFrontRight + ',' + convertedIRBack + ',' + convertedIRBackRight + ']';
    Serial.print(outputString);

    Wire.requestFrom(8, 1);
    if(Wire.available()) {
        velocity = Wire.read();   
     }

    //Serial.println(velocity);   

    // Both uppercase and lowercase left, right, forward and back work.
    inputString.toUpperCase();

    //myMotor.writeMicroseconds(1500);
    //myServo.writeMicroseconds(1105);

    // If input == D, turn right --700? Very sharp
    // -- 800 relatively sharp
    if (steeringVal > 0) {
      myServo.writeMicroseconds(775);
      /*strip.setPixelColor(0,  204, 102, 0);
      strip.setPixelColor(1,  204, 102, 0);
      strip.setPixelColor(14,  204, 102, 0);
      strip.setPixelColor(15,  204, 102, 0);
      strip.show();
      delay(50);
      strip.setPixelColor(0,  0, 0, 0);
      strip.setPixelColor(1,  0, 0, 0);
      strip.setPixelColor(14,  0, 0, 0);
      strip.setPixelColor(15,  0, 0, 0);
      strip.show();*/
    }
    // If input == A, turn left --1510?
    // -- 1450
    else if (steeringVal < 0) {
      myServo.writeMicroseconds(1425);
      /*strip.setPixelColor(6,  204, 102, 0);
      strip.setPixelColor(7,  204, 102, 0);
      strip.setPixelColor(8,  204, 102, 0);
      strip.setPixelColor(9,  204, 102, 0);
      strip.show();
      delay(50);
      strip.setPixelColor(6,  0, 0, 0);
      strip.setPixelColor(7,  0, 0, 0);
      strip.setPixelColor(8,  0, 0, 0);
      strip.setPixelColor(9,  0, 0, 0);
      strip.show();*/
    } else {  
      if(steeringVal == 0){
          myServo.writeMicroseconds(1105);
        }    
    }
   
    // If speedval is higher than 0, drive forward
    if (speedVal > 0) {
      if(velocity >= 2){
        myMotor.writeMicroseconds(1555);
      }
      if(velocity < 2){
        myMotor.writeMicroseconds(1565);
      }
      if(speedVal == 0){
        myMotor.writeMicroseconds(1500);
      }
      if (speedVal == 2) {
        myMotor.writeMicroseconds(1200);
        myMotor.writeMicroseconds(1500);
      }
      if (speedVal == 3) {
        myMotor.writeMicroseconds(1555);
        myMotor.writeMicroseconds(1500);
      }
    }
    
    // If speedval is lower than 0, drive backwards
    if (speedVal < 0) {
      if(velocity > 2){
        myMotor.writeMicroseconds(1250);
        
      }else{
        myMotor.writeMicroseconds(1200);
      }
    }
    //inputString = "";
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    int firstDelim;
    int lastDelim;
    int middleDelim;
    String speedStr, steeringStr;


    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // so the main loop can do something about it:
    while (inputString.startsWith("{") && inputString.endsWith("}")) {

      firstDelim = inputString.indexOf("{");
      middleDelim = inputString.indexOf(",");
      lastDelim = inputString.indexOf("}");
      speedStr = inputString.substring(firstDelim + 1, middleDelim);
      steeringStr = inputString.substring(middleDelim + 1, lastDelim);
      inputString = "";
    }
    speedVal = (double) speedStr.toInt();
    steeringVal = (double) steeringStr.toInt();
  }
}

// Interrupt sequence sets the flag variable to 1
void stopAll() {
  flag = 1;
}

int IRConversion(int value){
    int convertedValue;
    convertedValue = (2914/value+5)-1;
    return convertedValue;    
}


