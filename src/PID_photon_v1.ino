// This #include statement was automatically added by the Particle IDE.
#include <SparkIntervalTimer.h>
#include <ThingSpeak.h>
#include "PID_v1.h"
#include "OneWire.h"
#include "DS18.h"


#define DEBUG_TRUE FALSE

// Input pin where DS18B20 is connected
#define DS18PIN          D3
#define RELAYPIN         D6
#define LEDPIN           D7
#define MAX_READERROR    30
#define READ_INTERVAL    1000
#define PUBLISH_INTERVAL 30000


unsigned long myChannelNumber   = 228423;
const char * myWriteAPIKey      = "A4I3XH61191GVIQH";

typedef uint8_t DeviceAddress[8];
DeviceAddress SENSOR1           = {0x28, 0xFF, 0x27, 0x16, 0xA1, 0x16, 0x03, 0x29};
DeviceAddress SENSOR2           = {0x28, 0xFF, 0xA1, 0x02, 0xA1, 0x16, 0x03, 0x48};
//Define Variables we'll be connecting to
double Setpoint, Input, Input2, Output, Kp, Ki, Kd;
int windowSize                  = 3000;
int crcErrorCount               = 0;
bool inputselect1               = TRUE;
int eStop                       = 0;
unsigned long readMillis        = 0;
unsigned long publishMillis     = 0;
unsigned long windowStartTime;
String controlMode;
String PIDaction = "P_ON_E";


TCPClient client;
DS18 sensor(DS18PIN);
PID myPID(&Input, &Output, &Setpoint,80,5,2, PID::P_ON_E, PID::DIRECT);


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



int set_Setpoint(String command) {
    if (isValidNumber(command)){
        Setpoint = (double) command.toInt();
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
   Particle.variable("E-STOP",         eStop);
   Particle.variable("CRCERR",         crcErrorCount);

   // ParticleCloud: functions
   Particle.function("setpoint",       set_Setpoint);
   Particle.function("action",         set_Action);
   Particle.function("tune",           set_Tunings);
   Particle.function("win_size",       set_WindowSize);
   Particle.function("reboot",         reBoot);
   Particle.function("control",        setControlSensor);

   ThingSpeak.begin(client);

    //initialize the variables we're linked to
   Setpoint  = 60;

   //tell the PID to range between 0 and the full window size
   myPID.SetOutputLimits(0, windowSize);
   myPID.SetMode(PID::AUTOMATIC);

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
   else{

       myPID.Compute();

       /************************************************
       * turn the output pin on/off based on pid output
       ************************************************/
       if (now - windowStartTime > windowSize) {
           windowStartTime += windowSize;
       }

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
