/*GY-273 Compass Module  ->  Arduino
 * VCC  -> VCC  (See Note Below)
 * GND  -> GND
 * SCL  -> A5/SCL, (Use Pin 21 on the Arduino Mega)
 * SDA  -> A4/SDA, (Use Pin 20 on the Arduino Mega)
 * DRDY -> Not Connected (in this example)
 *
 *Ultrasonic sensor Pins:
*       VCC: +5VDC
*      Trig : Trigger (INPUT) - Pin11
*     Echo: Echo (OUTPUT) - Pin 12
*     GND: GND
 */
 

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h> //Wire Librarey for I2C communication 
#include <MechaQMC5883.h> //QMC5883 Library is added since mine is QMC583 and not HMC5883
#include <Servo.h>
#include <SPI.h> //for the SD card module
#include <SD.h> // for the SD card
#define ARDUINO_USD_CS 53 // SD card CS pin (pin 53 on the Logger board)

 //------------------------- Log File Defintions --------------------------------//

// Keep in mind, the SD library has max file name lengths of 8.3 - 8 char prefix,
// and a 3 char suffix.
// Our log files are called "gpslogXX.csv, so "gpslog99.csv" is our max file.
#define LOG_FILE_PREFIX "gpslog" // Name of the log file.
#define MAX_LOG_FILES 100 // Number of log files that can be made
#define LOG_FILE_SUFFIX "csv" // Suffix of the log file
char myFileName[13]; // Char string to store the log file name
// Data to be logged:
#define LOG_COLUMN_COUNT 8
char * log_col_names[LOG_COLUMN_COUNT] = {
  "longitude", "latitude", "altitude", /*"speed", "course", "date", "time", */ "satellites"
}; // log_col_names is printed at the top of the file.

const int chipSelect = 53;

 // miso on pin 50
 //mosi on pin 51
 // Sck on pin 52
 // cs on pin 53


int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a file to store the data
File myFile;

MechaQMC5883 qmc; //Create an object name for the snsor, I have named it as qmc
// Create a software serial port called "gpsSerial"
//SoftwareSerial gpsSerial(RXPin, TXPin);

Servo myservo;  // create servo object to control a servo

#include <math.h>

//------------------------DC motor variables------------------
int In1 = 12;
int In2 = 11;
int En = 10;

//-------------buzzer and led variables------------------------

int buzzerPin = 25;
int ledPin = 23;

//-------------longitude and lattitude variables---------------
float way_Lat;
float way_Lon;
float latR1;
float latR2;
float lonR1;
float lonR2;
float dlon;
float dlat;
float a;
float e;
int distance;
int reqBear;
int heading;
float R = 6371.00;
float toDegrees = 57.295779;
char sb[10];


// data over bluetooth variable
char data;
int start_Trip, end_Trip;
int motion;
int Count;

float wayPoints_Lat[25] = {
     //.....................latitude point system coordinates comes here.........//
     };
  
  float wayPoints_Lon[25] = {
      //.....................latitude point system coordinates comes here.........//
       };


//--------------defining distance sensor pins---------------
int trigPin1 = 2;    // Trigger1
int echoPin1 = 3;    // Echo1
long Obstacledistance;
long duration1;


void setup() {
   Serial2.begin(9600); // connect to Rx = 17, Tx = 16
   Serial1.begin(9600); //Begin Serial Communication Rx = 19, Tx = 18
   Serial.begin(9600);
   Wire.begin(); //Begin I2C communication 
   Serial.begin(9600); // connect serial
   qmc.init(); //Initialise the QMC5883 Sensor 

   // Start the software serial port at the GPS's default baud
   // gpsSerial.begin(GPSBaud);
   // put your setup code here, to run once:
   myservo.attach(9);  // attaches the servo on pin 9 to the servo object
   myservo.write(90); 
   delay(15);
   
   //initialize distance sensor inputs and outputs
   pinMode(trigPin1, OUTPUT);
   pinMode(echoPin1, INPUT);
   
   //initialize ledPin and buzzerPin
   pinMode(ledPin, OUTPUT);
   pinMode(buzzerPin, OUTPUT);

   //initialize l298N motor controller
   pinMode(In1, OUTPUT);
   pinMode(In2, OUTPUT);
   pinMode(En, OUTPUT);

  //----------------turn the motor off at initial stage-------------
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);

   if (!SD.begin(ARDUINO_USD_CS))
 {
   Buzz(1000);
 }
 updateFileName(); // Each time we start, create a new file, increment the number
 printHeader(); // Print a header at the top of the new file
  
}



void loop() {
  getGPSdata();
  getCompass_Heading();
  obstacleSensor1();
  decodeBluetooth();
  Count = start_Trip;
  way_Lat = wayPoints_Lat[Count];
  way_Lon = wayPoints_Lon[Count];

  //if(gps.satellites.value() >= 3){
  if(gps.location.isValid()){
    Blink(600,600);
    if(distance <= 2.5){
           Buzz(1000);
           logGPSData(); 
           Count++;
               }
       if(motion == 4){
        if  (Obstacledistance <= 80){
                delay(350);
                Stop();
                  steerLeft();       
                  Count++;  
              }
        
             else if ((reqBear - heading) > 15){ 
                  steerRight();
                }
             
             else if ((reqBear - heading) < -15){ 
                  steerLeft();
                }
                
             else if (Obstacledistance > 80){
             myservo.write(90); 
             driveStraight;
             
             }
             
             }  
         
                  else if((motion == 3) || (Count = end_Trip)){
                      Stop();
                       myservo.write(90); 
                      delay(15);
          }
          }
        
}

      
//--------------------function to get bearing, distance and GPS data---------------------------------
void getGPSdata(){
  while (Serial1.available()){
      if (gps.encode(Serial1.read())){   
        if (gps.location.isValid())
          {
             calcDist((gps.location.lat()), (gps.location.lng()));
//             Serial.print("Latitude: ");
//             Serial.println(gps.location.lat(), 6);
//             Serial.print("Longitude: ");
//             Serial.println(gps.location.lng(), 6);
//             Serial.print(" GPS data. Sats: ");
//             Serial.println(gps.satellites.value());

             
          }
          else
          {
             //Serial.println("Location: Not Available");
          }
      }
  }
           // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
          if (millis() > 5000 && gps.charsProcessed() < 10)
          {
            //Serial.println("No GPS detected");
            Buzz(5000);
            while(true);
          }
}




/*-----------------------------Distance & Bearing Calculator---------------------------------*/
void calcDist(float current_Lat, float current_Lon){ //This is a haversine based distance calculation formula
      
  //This portion converts the current and destination GPS coords from decDegrees to Radians
  lonR1 = current_Lon*(PI/180);
  lonR2 = way_Lon*(PI/180);
  latR1 = current_Lat*(PI/180);
  latR2 = way_Lat*(PI/180);
  //This portion calculates the differences for the Radian latitudes and longitudes and saves them to variables
  dlon = lonR2 - lonR1;
  dlat = latR2 - latR1;
  
  //This portion is the Haversine Formula for distance between two points. Returned value is in Meters
  a = (sq(sin(dlat/2))) + cos(latR1) * cos(latR2) * (sq(sin(dlon/2)));
  e = 2 * atan2(sqrt(a), sqrt(1-a)) ;
  distance = (R * e * 1000); //distance in meters

  //Serial.println();
  //Serial.print("Distance to destination(Meters): ");
  //Serial.println(a);
  //Serial.println(e);
  //Serial.println(distance);
  //Serial.println();
  
  //This portion is the Haversine Formula for required bearing between current location and destination. Returned value is in Degrees
  float x = cos(latR2)*sin(lonR2-lonR1); //calculate x
  
  //Serial.print("X = ");
  //Serial.println(x, 6);

  float y = cos(latR1)*sin(latR2)-sin(latR1)*cos(latR2)*cos(lonR2-lonR1); //calculate y

  //Serial.print("Y = ");
  //Serial.println(y, 6);
  float brRad = atan2(x, y); //return atan2 result for bearing. Result at this point is in Radians

  //Serial.print("atan2(x, y) (Radians) = ");
  //Serial.println(brRad, 6);

  reqBear = toDegrees*brRad;
  //Serial.print("Bearing: ");
  //Serial.println(reqBear, 4);


  if(reqBear<0){

  reqBear+=360;   //if the heading is negative then add 360 to make it positive

}

//Serial.print("Heading from GPS: ");
//Serial.println(reqBear);   // print the heading.
 
}



//---------------------------function to get heading from the electronic compass-----------------------------------
void getCompass_Heading(){
   int x,y,z;
  qmc.read(&x,&y,&z); //Get the values of X,Y and Z from sensor 
  
  heading=atan2(x, y)/0.0174532925; //Calculate the degree using X and Y parameters with this formulae 

 //Convert result into 0 to 360
  if(heading < 0) 
  heading+=360;
  heading = 360-heading;
  
  //Serial.println(heading); //Print the value of heading in degree for debugging 
}



//------------------------------function to get the distance from the first ultrasonic sensor-----------------------
void obstacleSensor1(){
   // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin1, INPUT);
  duration1 = pulseIn(echoPin1, HIGH);
  
  // -------------Convert the time into a distance---------------
  Obstacledistance = (duration1 / 2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
//  Serial.print(Obstacledistance);
//  Serial.print("cm");
//  Serial.println();
//  
//  delay(250);
}


void decodeBluetooth(){

  if(Serial2.available() > 0)
   {
      data = Serial2.read();
//      Serial.print(data);
//      Serial.print("\n");

      switch(data){
        case '0':
          motion = 4;
          //Serial.println(motion);
          break;
          
        case '1':
          motion = 3;
          //Serial.println(motion);
          break;
          
        case '2':
          start_Trip = 0;
          //Serial.println(start_Trip);
          break;

        case '3':
          start_Trip = 1;
          //Serial.println(start_Trip);
          break;

        case '4':
          start_Trip = 2;
          //Serial.println(start_Trip);
          break;

        case '6':
          end_Trip = 8;
          //Serial.println(end_Trip);
          break;

        case '7':
          end_Trip = 15;
          //Serial.println(end_Trip);
          break;

        case '8':
          end_Trip = 18;
          //Serial.println(end_Trip);
          break;

        case '9':
          end_Trip = 22;
          //Serial.println(end_Trip);
          break;

         case 'a':
          end_Trip = 23;
          //Serial.println(end_Trip);
          break;

         case 'b':
          end_Trip = 24;
          //Serial.println(end_Trip);
          break;
          
       default:
       start_Trip = 0;
       end_Trip = 24;

      }
  
}
}

void steerLeft(){
   myservo.write(125); 
   delay(15);
   digitalWrite(In1, HIGH);
   digitalWrite(In2, LOW);
   analogWrite(En, 150);
}


void steerRight(){
   myservo.write(60); 
   delay(15);
   digitalWrite(In1, HIGH);
   digitalWrite(In2, LOW);
   analogWrite(En, 150);
}


void driveStraight(){
   myservo.write(90); 
   delay(15);
   digitalWrite(In1, HIGH);
   digitalWrite(In2, LOW);
   analogWrite(En, 250);
}


void reverseRight(){
   myservo.write(125); 
   delay(15);
   digitalWrite(In1, LOW);
   digitalWrite(In2, HIGH);
   analogWrite(En, 150);
  
}


void reverseLeft(){
   myservo.write(60); 
   delay(15);
   digitalWrite(In1, LOW);
   digitalWrite(In2, HIGH);
   analogWrite(En, 150);
}


void reverseStraight(){
   myservo.write(90); 
   delay(15);
   digitalWrite(In1, LOW);
   digitalWrite(In2, HIGH);
   analogWrite(En, 250);
}

void Stop(){
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
}


void Blink(int H, int L){
  digitalWrite(ledPin, HIGH);
  delay(H);
  digitalWrite(ledPin, LOW);
  delay(L);
}


void Buzz(int H){
  digitalWrite(buzzerPin, HIGH);
  delay(H);
  digitalWrite(buzzerPin, LOW);
}


byte logGPSData()
{
  File myFile = SD.open(myFileName, FILE_WRITE); // Open the log file

  if (myFile)
  { // Print longitude, latitude, altitude (in feet), speed (in mph), course
    // in (degrees), date, time, and number of satellites.
    myFile.print(gps.location.lng(), 6);
    myFile.print(',');
    myFile.print(gps.location.lat(), 6);
    myFile.print(',');
    myFile.print(gps.altitude.feet(), 1);
    myFile.print(',');
   /* myFile.print(tinyGPS.speed.mph(), 1);
    myFile.print(',');tinyGPS
    myFile.print(tinyGPS.course.deg(), 1);
    myFile.print(',');
    myFile.print(tinyGPS.date.value());
    myFile.print(',');
    myFile.print(tinyGPS.time.value());
    myFile.print(',');*/
    myFile.print(gps.satellites.value());
    myFile.println();
    myFile.close();

    return 1; // Return success
  }

  return 0; // If we failed to open the file, return fail

}

// printHeader() - prints our eight column names to the top of our log file
void printHeader()
{
  File myFile = SD.open(myFileName, FILE_WRITE); // Open the log file

  if (myFile) // If the log file opened, print our column names to the file
  {
    int i = 0;
    for (; i < LOG_COLUMN_COUNT; i++)
    {
      myFile.print(log_col_names[i]);
      if (i < LOG_COLUMN_COUNT - 1) // If it's anything but the last column
        myFile.print(','); // print a comma
      else // If it's the last column
        myFile.println(); // print a new line
    }
    myFile.close(); // close the file
  }
}

 //updateFileName(); //- Looks through the log files already present on a card,
// and creates a new file with an incremented file index.
void updateFileName()
{
  int i = 0;
  for (; i < MAX_LOG_FILES; i++)
  {
    memset(myFileName, 0, strlen(myFileName)); // Clear myFileName string
    // Set myFileName to "gpslogXX.csv":
    sprintf(myFileName, "%s%d.%s", LOG_FILE_PREFIX, i, LOG_FILE_SUFFIX);
    if (!SD.exists(myFileName)) // If a file doesn't exist
    {
      break; // Break out of this loop. We found our index
    }
    else // Otherwise:
    {
      Serial.print(myFileName);
      Serial.println(" exists"); // Print a debug statement
    }
  }
  Serial.print("File name: ");
  Serial.println(myFileName); // Debug print the file name
}
