// This #include statement was automatically added by the Particle IDE.
#include <SparkIntervalTimer.h>
#include <ThingSpeak.h>
#include "PID_v1.h"
#include "OneWire.h"
#include "DS18.h"
#include "PID-AutoTune.h"


#define DEBUG_TRUE FALSE


// ************************************************
// Pin definitions
// ************************************************
#define DS18PIN    D3
#define RELAYPIN   D6
#define LEDPIN     D7


// ************************************************
// ThingSpeak Variables and constants
// ************************************************
#define READ_INTERVAL           1000
#define PUBLISH_INTERVAL        30000
unsigned long myChannelNumber   = xxx;
const char * myWriteAPIKey      = "xxx";
TCPClient client;


// ************************************************
// Probe ERROR detetion Variables and constants
// ************************************************
#define MAX_READERROR    30
int eStop                = 0;
int crcErrorCount        = 0;


// ************************************************
// Probe Addressing, Variables, and constants
// ************************************************
DS18 sensor(DS18PIN);
typedef uint8_t DeviceAddress[8];
DeviceAddress SENSOR1           = {0x28, 0xFF, 0x27, 0x16, 0xA1, 0x16, 0x03, 0x29};
DeviceAddress SENSOR2           = {0x28, 0xFF, 0xA1, 0x02, 0xA1, 0x16, 0x03, 0x48};


// ************************************************
// PID Variables and constants
// ************************************************
double Setpoint, Input, Input2, Output, Kp, Ki, Kd;
int windowSize              = 3000;
bool inputselect1           = TRUE;
unsigned long readMillis    = 0;
unsigned long publishMillis = 0;
unsigned long windowStartTime;



String controlMode;
String PIDaction = "P_ON_E";
String PIDmode   = "MANUAL";
String isTuning  = "FALSE";

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, PID::P_ON_E, PID::DIRECT);


// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;
double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;
boolean tuning = false;
PID_ATune aTune(&Input, &Output);



void publishThingspeak(double a, double b, unsigned long t) {

    if (t - publishMillis >= PUBLISH_INTERVAL) {

        publishMillis = t;

        if (inputselect1) {
            ThingSpeak.setField(5, String(a));
            ThingSpeak.setField(6, String(b));
        }
        else {
            ThingSpeak.setField(5, String(b));
            ThingSpeak.setField(6, String(a));
        }

        ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    }
    delay(500);
}


int startAT(String command){
  if (isValidNumber(command)){
      int a = command.toInt();
      if (a == 1) {
        StartAutoTune();
        isTuning = "TRUE";
      } return 1;
    }
  else {
    return 0;
  }
}


int set_Setpoint(String command) {
    if (isValidNumber(command)){
        Setpoint = (double) command.toInt();
        SaveParameters();
        return 1;
    }
    else{
        return 0;
    }

}


int set_Output(String command) {
    if (isValidNumber(command)){
        Output = (double) command.toInt();
        return 1;
    }
    else{
        return 0;
    }

}


int set_Tunings(String command) {

    if (isValidPID(command)){
        int commaIndex = command.indexOf(',');
        //  Search for the next comma just after the first
        int secondCommaIndex = command.indexOf(',', commaIndex + 1);

        String kpVal = command.substring(0, commaIndex);
        String kiVal = command.substring(commaIndex + 1, secondCommaIndex);
        String kdVal = command.substring(secondCommaIndex + 1); // To the end of the string

        // Convert to int
        double p = (double) kpVal.toFloat();
        double i = (double) kiVal.toFloat();
        double d = (double) kdVal.toFloat();

        myPID.SetTunings(p, i, d);
        Kp = myPID.GetKp();
        Ki = myPID.GetKi();
        Kd = myPID.GetKd();
        SaveParameters();

        return 1;
    }
    else{
        return 0;
    }
}


int set_WindowSize(String command) {
    if (isValidNumber(command)){
        windowSize = (double) command.toInt();
        myPID.SetOutputLimits(0, windowSize);
        return 1;
    }
    else {
        return 0;
    }
}


int set_Action(String command) {
    if (isValidNumber(command)){
        bool a = command.toInt();

        if (a) {
            myPID.SetAction(PID::P_ON_E);
            PIDaction = "P_ON_E";
        }
        else if (!a) {
            myPID.SetAction(PID::P_ON_M);
            PIDaction = "P_ON_M";
        }

        return 1;
    }
    else {
        return 0;
    }
}


int set_Mode(String command) {
    if (isValidNumber(command)){
        bool a = command.toInt();

        if (a == 1) {
            myPID.SetMode(PID::AUTOMATIC);
            PIDmode = "AUTOMATIC";
        }
        if (a == 0) {
            myPID.SetMode(PID::MANUAL);
            PIDmode = "MANUAL";
        }

        return 1;
    }
    else {
        return 0;
    }
}

int reBoot(String command) {
    if  (command.equals("reboot")) System.reset();
    else return 0;
}


int setControlSensor(String command) {
    if ((command.equals("1")) || (command.equals("2")))
    {
        if (command.equals("1"))
        {
            inputselect1 = TRUE;
            return 1;
        }
        if (command.equals("2"))
        {
            inputselect1 = FALSE;
            return 1;
        }
    }
    else
    {
        return 0;
    }
}


bool isValidPID(String str){
   boolean isNum=false;
   for(byte i=0;i<str.length();i++)
   {
       isNum = isDigit(str.charAt(i)) || str.charAt(i) == '+' || str.charAt(i) == '.' || str.charAt(i) == '-' || str.charAt(i) == ',';
       if(!isNum) return false;
   }
   return isNum;
}


bool isValidNumber(String str){
   boolean isNum=false;
   for(byte i=0;i<str.length();i++)
   {
       isNum = isDigit(str.charAt(i));
       if(!isNum) return false;
   }
   return isNum;
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************
void StartAutoTune()
{
   // REmember the mode we were in
   AutoTuneHelper(true);

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune() {
   tuning = false;
   isTuning = "FALSE";
   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   AutoTuneHelper(false);

   // Persist any changed parameters to EEPROM
   SaveParameters();
}


void AutoTuneHelper(boolean start) {
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode((PID::mode_t)ATuneModeRemember);

}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters() {
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters() {
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);

   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value) {
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address) {
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

// Setup
void setup() {

   // Start serial on USB
   //Serial.begin(9600);

   pinMode(RELAYPIN, OUTPUT);
   pinMode(LEDPIN, OUTPUT);

   // ParticleCloud: variables
   Particle.variable("setpoint",       Setpoint);
   Particle.variable("output",         Output);
   Particle.variable("Kp",             Kp);
   Particle.variable("Ki",             Ki);
   Particle.variable("Kd",             Kd);
   Particle.variable("windowsize",     windowSize);
   Particle.variable("control",        controlMode);
   Particle.variable("action",         PIDaction);
   Particle.variable("mode",           PIDmode);
   Particle.variable("estop",          eStop);
   Particle.variable("crcerr",         crcErrorCount);
   Particle.variable("istune",         isTuning);

   // ParticleCloud: functions
   Particle.function("Setpoint",       set_Setpoint);
   Particle.function("Action",         set_Action);
   Particle.function("Tune",           set_Tunings);
   Particle.function("Output",         set_Output);
   Particle.function("Win_Size",       set_WindowSize);
   Particle.function("Reboot",         reBoot);
   Particle.function("Control",        setControlSensor);
   Particle.function("AutoTune",       startAT);
   Particle.function("Mode",           set_Mode);

   ThingSpeak.begin(client);

   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);
   //tell the PID to range between 0 and the full window size
   myPID.SetOutputLimits(0, windowSize);
   myPID.SetMode(PID::MANUAL);

   windowStartTime = millis();;

   Kp = myPID.GetKp();
   Ki = myPID.GetKi();
   Kd = myPID.GetKd();
}

// Loop
void loop() {

   unsigned long now = millis();
   // Send event to the ParticleCloud
   if (now - readMillis >= READ_INTERVAL) {
       readMillis = now;

       if (inputselect1) {
           if (controlMode != "Jacket"){
               controlMode = "Jacket";
               Particle.variable("JacketTemp", Input);
               Particle.variable("BatchTemp",  Input2);
           }

           if (sensor.read(SENSOR1)) {
               Input = sensor.fahrenheit();
               crcErrorCount = 0;
           }
           if (sensor.crcError()){
               ++crcErrorCount;
           }

           if (sensor.read(SENSOR2)) {
               Input2 = sensor.fahrenheit();
           }
       }
       else{
           if (controlMode != "Batch"){
               controlMode = "Batch";
               Particle.variable("JacketTemp", Input2);
               Particle.variable("BatchTemp",  Input);
           }
           if (sensor.read(SENSOR2)) {
               Input = sensor.fahrenheit();
               crcErrorCount = 0;
           }
           if (sensor.crcError()){
               ++crcErrorCount;
           }

           if (sensor.read(SENSOR1)) {
               Input2 = sensor.fahrenheit();
           }
       }
   }

   if (crcErrorCount >= MAX_READERROR){
     eStop = 1;
     Input = 999;
     Output = 0;
     digitalWrite(RELAYPIN,LOW);
     digitalWrite(LEDPIN,LOW);
   }
   if (tuning){ // run the auto-tuner

     if (aTune.Runtime()) // returns 'true' when done
     {
       FinishAutoTune();
     }
  }
  else { // Execute control algorithm

  myPID.Compute();
  }

       /************************************************
       * turn the output pin on/off based on pid output
       ************************************************/
  if (now - windowStartTime > windowSize) {
    windowStartTime += windowSize;

    if (Output > (now - windowStartTime)) {
      digitalWrite(RELAYPIN,HIGH);
      digitalWrite(LEDPIN,HIGH);
    }
    else{
      digitalWrite(RELAYPIN,LOW);
      digitalWrite(LEDPIN,LOW);
    }

      publishThingspeak(Input, Input2, now);

   }



}
