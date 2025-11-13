#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> 
#include <LiquidCrystal.h>

//#include <MPU6050.h>
//MPU6050 mpu;

#include <TinyGPS++.h>
TinyGPSPlus gps;

#include <SoftwareSerial.h>
// Choose two Arduino pins to use for software serial
int RXPin = 10;
int TXPin = 11;
int GSMBaud = 9600;

// Create a software serial port called "gpsSerial"
SoftwareSerial gsmSerial(RXPin, TXPin);
// Create a software serial port called "gpsSerial"
SoftwareSerial WiFiSerial(8, 9);

const int RS = 2;
const int E  = 3;
const int D4 = 4;
const int D5 = 5;
const int D6 = 6;
const int D7 = 7;

LiquidCrystal lcd(RS, E, D4, D5, D6, D7);

//Buzzer
int buzzer = 13;
int vibm = 12;

//Emergency Key
int ek = A3;
int key= 0;

//Temperature Sensor 
#define ONE_WIRE_BUS A0
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
String latlan = "0";

// Heart Beat Sensor Variables
int pulsePin = A1;     // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 0;      // pin to blink led at each beat
int fadePin  = 0;      // pin to do fancy classy fading blink at each beat
int fadeRate = 0;      // used to fade LED on with PWM on fadePin

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded! 
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat". 
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

// Regards Serial OutPut  -- Set This Up to your needs
static boolean serialVisual = true;   // Set to 'false' by Default.  Re-set to 'true' to see Arduino Serial Monitor ASCII Visual Pulse 

volatile int rate[10];                      // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P = 512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

// MPU 6050
int pitch = 0;
int roll = 0;
int msg = 0;
int count = 0;
int count1, count2, count3, count4 = 0;


void setup() {
  // put your setup code here, to run once:
  analogReference(INTERNAL);
  pinMode(buzzer, OUTPUT);
  pinMode(vibm, OUTPUT);

  //pinMode(led_pin,OUTPUT);
  //pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
  //pinMode(fadePin,OUTPUT);          // pin that will fade to your heartbeat!

  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
                                    // IF YOU ARE POWERING The Pulse Sensor AT VOLTAGE LESS THAN THE BOARD VOLTAGE, 
                                    // UN-COMMENT THE NEXT LINE AND APPLY THAT VOLTAGE TO THE A-REF PIN
                                    //   analogReference(EXTERNAL);   
              

  gsmSerial.begin(GSMBaud);
  WiFiSerial.begin(9600);
    
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("   PERSON SAFETY   ");
  lcd.setCursor(0,1);
  lcd.print("ACTIVITY MONITORING");
  delay(1800);
  
  //Temp Sensor
  sensors.begin();
  Serial.begin(9600);         // Setting the baudrate at 9600
  /*while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
  mpu.setIntFreeFallEnabled(false);  
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);
  mpu.setMotionDetectionThreshold(2);
  mpu.setMotionDetectionDuration(5);
  mpu.setZeroMotionDetectionThreshold(4);
  mpu.setZeroMotionDetectionDuration(2);*/

  digitalWrite(buzzer,HIGH);
  delay(100);
  digitalWrite(vibm,HIGH);
  delay(50);
  digitalWrite(vibm,LOW);
  digitalWrite(buzzer,LOW);
  lcd.clear();
}

void loop() {
  // put your main code here, to run repeatedly:
  // GPS Data
  while (Serial.available() > 0){
    gps.encode(Serial.read());
    if (gps.location.isUpdated()){
    }
  }

  // Temperature Sensor  
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures();
  // Calculate the average value from all "j1" readings.
  int tempvv = sensors.getTempCByIndex(0);

  // MPU Sensor Read normalized values 
 // Vector normAccel = mpu.readNormalizeAccel();
  // Calculate Pitch & Roll
 // pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
//  roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  key = digitalRead(ek);

  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.print(tempvv);
  lcd.print(" ");

  lcd.setCursor(4,0);
  lcd.print("H:");
  lcd.print(BPM);
  lcd.print(" ");

  lcd.setCursor(0,1);
  lcd.print("K:");
  lcd.print(key);
  lcd.print(" ");

  lcd.setCursor(10,0);
  lcd.print(gps.location.lat(), 1);
  lcd.print(",");
  lcd.print(gps.location.lng(), 1);


  
  if (tempvv > 100)
  {
    digitalWrite(buzzer,HIGH);
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(6,1);
    lcd.print("SDG.");

    Serial.print(tempvv);
    Serial.print(" ");
    Serial.print(BPM);
    Serial.print(" ");
    Serial.print(key);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(latlan);
    Serial.println();
    
    gsm_msg(1);
    gsm_msg(2);
    gsm_msg(3);
    gsm_call();
    delay(500);
    digitalWrite(buzzer,LOW);
  }
    
  if (key == 1)
  {
    digitalWrite(buzzer,HIGH);
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(6,1);
    lcd.print("SDG.");
    
    Serial.print(tempvv);
    Serial.print(" ");
    Serial.print(BPM);
    Serial.print(" ");
    Serial.print(key);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(latlan);
    Serial.println();
    
    gsm_msg(1);
    gsm_msg(2);
    gsm_msg(3);
    gsm_call();
    delay(200);
    digitalWrite(buzzer,LOW);
  }

  else
  {
    digitalWrite(buzzer,LOW);
  }

  Serial.print(tempvv);
  Serial.print(" ");
  Serial.print(BPM);
  Serial.print(" ");
  Serial.print(key);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(latlan);
  Serial.println();
  delay(500);

//Hear Beat Sensor 
serialOutput();  
  
  if (QS == true) // A Heartbeat Was Found
    {     
      // BPM and IBI have been Determined
      // Quantified Self "QS" true when arduino finds a heartbeat
      fadeRate = 255; // Makes the LED Fade Effect Happen, Set 'fadeRate' Variable to 255 to fade LED with pulse
      serialOutputWhenBeatHappens(); // A Beat Happened, Output that to serial.     
      QS = false; // reset the Quantified Self flag for next time    
    }
     
  ledFadeToBeat(); // Makes the LED Fade Effect Happen 
  delay(20); //  take a break
}

void gsm_msg(int a)
{
  lcd.setCursor(6,1);
  lcd.print("SDG.");
  delay(100);
  Serial.println("AT");
  delay(500);
  Serial.println("AT+CMGF=1");    //To send SMS in Text Mode
  digitalWrite(buzzer, HIGH);
  delay(2000);
  //8248561562
  if (a == 1){
    Serial.println("AT+CMGS=\"+918610551904\"\r"); // change to the phone number you using
    lcd.setCursor(6,1);
    lcd.print("SDG..1");
    delay(100);
    //Serial.println("1 SMS..."); 
  }
  else if(a == 2){
    Serial.println("AT+CMGS=\"+916383562433\"\r"); // change to the phone number you using 
    lcd.setCursor(6,1);
    lcd.print("SDG..2");
    delay(100);
  }
  else if(a == 3){
    Serial.println("AT+CMGS=\"+917708629206\"\r"); // change to the phone number you using 
    lcd.setCursor(6,1);
    lcd.print("SDG..3");
    delay(100);
  }
 
  
  digitalWrite(buzzer, LOW);
  delay(2000);
  lcd.setCursor(6,1);
  lcd.print("SDG..");
  //SMS Content
  Serial.print("Emergency Alert..! http://www.google.com/maps/place/");//the content of the message
  Serial.print(gps.location.lat(), 6);
  Serial.print(",");
  Serial.print(gps.location.lng(), 6);
  delay(500);
  lcd.setCursor(6,1);
  lcd.print("SDG...");
  delay(1000);
  Serial.println((char)26);//the stopping character
  delay(2000);
  digitalWrite(buzzer, HIGH);
  lcd.setCursor(6,1);
  lcd.print("SMS SENT....");
  delay(2000);
  lcd.setCursor(6,1);
  lcd.print("           ");
  delay(100);
  digitalWrite(buzzer, LOW); 
  
}

//gsm_call();
void gsm_call(){
  lcd.setCursor(6,1);
  lcd.print("CALL.");
  delay(100);
  Serial.println("AT");
  delay(500);
  Serial.println("ATD+ +918610551904;");
  delay(20000);
  lcd.setCursor(6,1);
  lcd.print("CA..");
  delay(1000);
  lcd.setCursor(6,1);
  lcd.print("C...");
  delay(1000);
  Serial.println("ATH");
  delay(2000);
  lcd.setCursor(6,1);
  lcd.print("CALL");
  delay(2000);
  lcd.setCursor(6,1);
  lcd.print("    ");
  delay(100);
}


void ledFadeToBeat()
{
   fadeRate -= 15;                         //  set LED fade value
   fadeRate = constrain(fadeRate,0,255);   //  keep LED fade value from going into negative numbers!
   //analogWrite(fadePin,fadeRate);          //  fade LED
}

void interruptSetup()
{     
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED      
} 


//Heart Beat Sensor Reading Calculation
void serialOutput()
{   // Decide How To Output Serial. 
 if (serialVisual == true)
  {  
     //arduinoSerialMonitorVisual('-', Signal);   // goes to function that makes Serial Monitor Visualizer
  } 
 else
  {
      //sendDataToSerial('S', Signal);     // goes to sendDataToSerial function
   }        
}

void serialOutputWhenBeatHappens()
{    
 if (serialVisual == true) //  Code to Make the Serial Monitor Visualizer Work
   {            
     //Serial.print("*** Heart-Beat Happened *** ");  //ASCII Art Madness
     //Serial.print("BPM: ");
     //Serial.println(BPM);
     //lcd.clear();
     
   }
 else
   {
     //sendDataToSerial('B',BPM);   // send heart rate with a 'B' prefix
     //sendDataToSerial('Q',IBI);   // send time between beats with a 'Q' prefix
   }   
}



ISR(TIMER2_COMPA_vect) //triggered when Timer2 counts to 124
{  
  cli();                                      // disable interrupts while we do this
  Signal = analogRead(pulsePin);              // read the Pulse Sensor 
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
                                              //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3) // avoid dichrotic noise by waiting 3/5 of last IBI
    {      
      if (Signal < T) // T is the trough
      {                        
        T = Signal; // keep track of lowest point in pulse wave 
      }
    }

  if(Signal > thresh && Signal > P)
    {          // thresh condition helps avoid noise
      P = Signal;                             // P is the peak
    }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250)
  {                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) )
      {        
        Pulse = true;                               // set the Pulse flag when we think there is a pulse
        //digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
        IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
        lastBeatTime = sampleCounter;               // keep track of time for next pulse
  
        if(secondBeat)
        {                        // if this is the second beat, if secondBeat == TRUE
          secondBeat = false;                  // clear secondBeat flag
          for(int i=0; i<=9; i++) // seed the running total to get a realisitic BPM at startup
          {             
            rate[i] = IBI;                      
          }
        }
  
        if(firstBeat) // if it's the first time we found a beat, if firstBeat == TRUE
        {                         
          firstBeat = false;                   // clear firstBeat flag
          secondBeat = true;                   // set the second beat flag
          sei();                               // enable interrupts again
          return;                              // IBI value is unreliable so discard it
        }   
      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable    

      for(int i=0; i<=8; i++)
        {                // shift data in the rate array
          rate[i] = rate[i+1];                  // and drop the oldest IBI value 
          runningTotal += rate[i];              // add up the 9 oldest IBI values
        }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
  }

  if (Signal < thresh && Pulse == true)
    {   // when the values are going down, the beat is over
      //digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
      Pulse = false;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
      P = thresh;                            // reset these for next time
      T = thresh;
    }

  if (N > 2500)
    {                           // if 2.5 seconds go by without a beat
      thresh = 512;                          // set thresh default
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
      firstBeat = true;                      // set these to avoid noise
      secondBeat = false;                    // when we get the heartbeat back
    }

  sei();                                   // enable interrupts when youre done!
}// end isr
