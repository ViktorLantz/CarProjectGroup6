#include <Adafruit_NeoPixel.h>
#include <SonarSRF02.h>
#include <SonarSRF08.h>
#include <SonarSRF.h>
#include <Servo.h>

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


int ch1, ch2, ch3, ch4;
volatile int flag;
int counter;
int midPoint;


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
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
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
  

  // USSensors connecting and register address location
  USFront.connect(MAIN_08_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER);
  //isConnected("SRF08Front", USFront.getSoft());
  USFrontRight.connect(MAIN_08_ADDRESS2, GAIN_REGISTER, LOCATION_REGISTER);
  //isConnected("SRF08FrontRight", USFrontRight.getSoft());

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
   
   if(flag == 1){

    // Get the values from US sensor front and front right, call distance function to print
    // Out the actual value from the sensors.
    int sensorReading = 0;
    sensorReading = USFront.getRange(unit);
    //distance("sensor", sensorReading); 
    //Serial.print(sensorReading);
    sensorReading = USFrontRight.getRange(unit);
    //distance("sensor", sensorReading);
    //Serial.print(sensorReading);

    // IR sensor values. Close == High && above 600 is too close to the sensors so it bugsout.
    IRFrontRight = analogRead(IRSensorFrontRight);
    Serial.print("ir");
    Serial.print(IRFrontRight);
    IRBackRight = analogRead(IRSensorBackRight);
    //Serial.print(IRBackRight);
    IRBack = analogRead(IRSensorBack);
    //Serial.print(IRBack);
    String us = "[{u";
    String ir = "{i";

     // ch1 = servo, ch2 = motor, ch3 & ch4 = encoder
     ch1 = pulseIn(3, HIGH);
     ch2 = pulseIn(5, HIGH);
     //ch3 = digitalRead(12);
     //ch4 = digitalRead(13);
     
     myServo.writeMicroseconds(1105);
     myMotor.writeMicroseconds(1500);
      
      // Turn Right 1400
      if(ch1 < 1450 && ch1 > 500){
        myServo.writeMicroseconds(1400);
      }
      // Turn Left 800
      if(ch1 > 1560){
        myServo.writeMicroseconds(800);
        
      }
      // Drive Forward
      if(ch2 > 1560){
        myMotor.writeMicroseconds(1580);
      }
      // Drive Backward
      if(ch2 < 1440 && ch2 > 500){
        myMotor.writeMicroseconds(1200);
      }

      // Motor and Servo microsecond values errorTesting prints.
      //Serial.print("This is channel 1:");
      //Serial.println(ch1);
      //Serial.print("This is channel 2:");
      //Serial.println(ch2);

      // Testing Encoder Values
      //Serial.print("This is channel 3:");
      //Serial.println(ch3);
      //Serial.print("This is channel 4:");
      //Serial.println(ch4);
      

      // Ensure that the receiver is turned off by checking that the value of
      // ch1 == 0 for 5 runs of the loop.
      // Some testing of the average midpoint value of the servo (1105 for current knob settings)
      if(ch1 == 0){  
        counter++;
        
        //midPoint += ch1;
      }else{
        counter = 0;
      }
      
      if(counter > 5){
        flag = 0;
        
        //midPoint = midPoint/10;
        //Serial.print("The middle point of 10 values:");
        //Serial.println(midPoint);
        //counter = 0;
        //midPoint = 0;
      }
   }else{  
    
    // Get the values from US sensor front and front right, call distance function to print
    // Out the actual value from the sensors.
    int frontSensorReading = 0;
    int frontRightSensorReading = 0;
    frontSensorReading = USFront.getRange(unit);
    //distance("sensor", sensorReading);
    //Serial.print("us"); 
    //Serial.print(sensorReading);
    //Serial.print("{");
    frontRightSensorReading = USFrontRight.getRange(unit);
    //distance("sensor", sensorReading);
    //Serial.print(sensorReading);

    // IR sensor values. Close == High && above 600 is too close to the sensors so it bugsout.
    IRFrontRight = analogRead(IRSensorFrontRight);
    //Serial.print("ir");
    //Serial.print(IRFrontRight);
    //Serial.print("{");
    IRBackRight = analogRead(IRSensorBackRight);
    //Serial.print(IRBackRight);
    //Serial.print("}");
    IRBack = analogRead(IRSensorBack);
    //Serial.print(IRBack);
    String us = "[{u";
    String ir = "{i";
    delay(20);
    outputString = us + frontSensorReading + ',' + frontRightSensorReading + '}' + ir + IRFrontRight + '%' + IRBack + '&' + IRBackRight + '}' + ']';
    Serial.print(outputString);
    // Both uppercase and lowercase left, right, forward and back work.
    inputString.toUpperCase();

    myMotor.writeMicroseconds(1500);
    myServo.writeMicroseconds(1105);
    
    // If input == D, turn right
    if(inputString == "D"){
      myServo.writeMicroseconds(900);
      strip.setPixelColor(0,  204, 102, 0);
      strip.setPixelColor(1,  204, 102, 0);
      strip.setPixelColor(14,  204, 102, 0);
      strip.setPixelColor(15,  204, 102, 0);
      strip.show();
      delay(100);
      strip.setPixelColor(0,  0, 0, 0);
      strip.setPixelColor(1,  0, 0, 0);
      strip.setPixelColor(14,  0, 0, 0);
      strip.setPixelColor(15,  0, 0, 0);
      strip.show();
    }
    // If input == A, turn left
    if(inputString == "A"){
      myServo.writeMicroseconds(1350);
      strip.setPixelColor(6,  204, 102, 0);
      strip.setPixelColor(7,  204, 102, 0);
      strip.setPixelColor(8,  204, 102, 0);
      strip.setPixelColor(9,  204, 102, 0);
      strip.show();
      delay(100);
      strip.setPixelColor(6,  0, 0, 0);
      strip.setPixelColor(7,  0, 0, 0);
      strip.setPixelColor(8,  0, 0, 0);
      strip.setPixelColor(9,  0, 0, 0);
      strip.show();
    }  
    // If input == M, turn to middle
    if(inputString == "M"){
      myServo.writeMicroseconds(1100);
    }
    // If input == W, drive forward
    if(inputString == "W"){
      myMotor.writeMicroseconds(1580);
    }
    // If input == S, drive backwards
    if(inputString == "S"){
      myMotor.writeMicroseconds(1200);
    }
    // If input == N, stand still
    if(inputString == "N"){
      myMotor.writeMicroseconds(1500);  
    }
    inputString = "";
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
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // so the main loop can do something about it:

  }
}

// Set the flag variable to 1
void stopAll(){
    flag = 1;
}

// Print out distance
void distance(String reference, int sensorReading) {
    Serial.print("Distance from " + reference + ": ");
    Serial.print(sensorReading);
    Serial.println(unit);
}

// Print out distance
void isConnected(String reference, int sensorSoft) {
    if (sensorSoft >= 0)
    {
        Serial.print("Sensor " + reference + " connected (");
        Serial.print(sensorSoft);
        Serial.println(")");
    }
    else
    {
        Serial.println("Sensor " + reference + " not detected");
    }
}

