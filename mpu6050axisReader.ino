#include <I2Cdev.h>

#include <MPU6050.h>

#include "math.h"

#include "Wire.h"


#define OUTPUT_READABLE_ACCELGYRO
#define KILLZONE 30
#define DEADZONE 5
/*-------------------------------------------------Variabili--------------------------*/
    MPU6050 accelgyro;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int i=0;
    double angoloGx=0;
    double angoloAx=0;
    double angoloFx=0;
    double kP=8, kI=0, kD=0, P, I, D, T_PID;
    double setPoint, error, prevError;
    double sampleTime;
    unsigned long time, prevTime;
/*------------------------------------------------------------------------------------*/

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    for(i=0;i<14;i++)
    {
      pinMode(i,OUTPUT);
    }
    i=0;
    
    // configure Arduino LED for
    Serial.print("       ax     |gx     |sec    |Filtered angle  |\n");
/*------------------------------------------------------------------------- imposto l'angolo inziale per il giroscopio -------------------*/
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    angoloAx=atan((ax)/(sqrt(pow(ay,2)+pow(az,2))));
    
    angoloAx=angoloAx*(180/M_PI);
    
    angoloGx = angoloAx;
    
    angoloFx = angoloAx;
    
    setPoint = 0;
    
    Serial.print(angoloAx); Serial.print("\t");
      
    Serial.print(angoloGx); Serial.print("\t");
    
}

/*------------------------------------------------------------------------- funzione che elabora la tensione e restituisce gli angoli ----*/
void GetTilt()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    angoloAx = atan((ax)/(sqrt(pow(ay,2)+pow(az,2))));
    
    angoloAx = angoloAx*(180/M_PI);
    
    angoloGx = angoloFx + ((gx/131) * (sampleTime/1000));
    
}
/*------------------------------------------------------------------------- elabora l'errore applicando il controllo PID -----------------*/
void ComputePID()
{
    error = angoloFx - setPoint;
    
    if(error > 0) 
    {
      kP = 7.6;
    }
    else
    {
      kP = 15;
    }
    
    P = error * kP;
    
    I = I + (error * sampleTime);
    I = I * kI;
       
    D = (error - prevError) * kD;

    prevError = error;    
    
    T_PID = abs(P + I + D);
    
    if(T_PID > 255)
    {
        T_PID = 255;    
    }
    
}
/*------------------------------------------------------------------------- aplica il filtro complementare sull'angolo misuarto ----------*/
void ComplementaryFilter()
{
    angoloFx = (0.75)*(angoloGx)+(0.25)*(angoloAx);
}
/*------------------------------------------------------------------------- ottiene il tempo di campionamento ----------------------------*/
void GetSampleTime()
{
  time = millis();
  
  sampleTime = time - prevTime;
  
  prevTime = time;
}
/*------------------------------------------------------------------------- funzione per il muovimento dei motori ------------------------*/
void DriveMotor()
{
    if((angoloFx < setPoint + DEADZONE) && (angoloFx > setPoint - DEADZONE))
    {
        digitalWrite(9, HIGH);
        digitalWrite(8, HIGH);
    }
    else
    {
        digitalWrite(9, LOW);
        digitalWrite(8, LOW);
        
        if(angoloFx > 0) 
        {
           digitalWrite(13, HIGH);
           digitalWrite(12, LOW);
        }
        else
        {
           digitalWrite(13, LOW);
           digitalWrite(12, HIGH);
        }
        
        if(abs(angoloFx) <= KILLZONE)
        {
            analogWrite(11, T_PID);
            analogWrite(3, T_PID);
        }
        else
        {
            analogWrite(11, 0);
            analogWrite(3, 0);
        }
    }
}

void loop() {
        GetSampleTime();
        GetTilt();
        ComplementaryFilter();
        ComputePID();
        DriveMotor();
        
        
        /*------------------------------------------------------------------- print dei risultati ------------------------------------------*/
        Serial.print("a/g:\t");
        
        Serial.print(angoloAx); Serial.print("\t");
        
        Serial.print(angoloGx); Serial.print("\t");
        
        Serial.print(angoloFx); Serial.print("\t");
        
        Serial.print(T_PID); Serial.print("\n");

}
