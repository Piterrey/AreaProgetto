#include <DateTime.h>
/*------------------------------------------Importante------------------------------*/
  /*       aggiunta alla libreria DateTime.cpp: #include <Arduino.h>   */
/*----------------------------------------------------------------------------------*/
  
#include <DateTimeStrings.h>

#include <I2Cdev.h>

#include <MPU6050.h>

#include "math.h"

#include "Wire.h"

//#include <MPU6050.h>

//#include <I2Cdev.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h




// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

//MPU6050 accelgyro(0x69); // <-- use for AD0 high




// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


/*#define LED_PIN_AX 13
#define LED_PIN_LAX 12
#define LED_PIN_AY 11
#define LED_PIN_LAY 10*/
/*-------------------------------------------------Variabili--------------------------*/
    MPU6050 accelgyro;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int i=0;
    double angoloGx=0;
    double angoloAx=0;
    time_t time=0;
    unsigned long nowTime=0;
    unsigned long oldTime=0;
    double fiAngx=0;
    double setPoint;
    double kP, kI, kD, P, I, D;
    double error, prevError, sumError, difError;
    double samplingT;

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
    pinMode(9, OUTPUT);   
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    
    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76        -2359        1688        0        0        0
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    
    */

    // configure Arduino LED for
    Serial.print("       ax     |gx     |sec    |Filtered angle  |\n");
    /*-------------------------------------------------------------------------- imposto l'angolo inziale per il giroscopio -----------------*/
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    angoloAx=atan((ax)/(sqrt(pow(ay,2)+pow(az,2))));
    
    angoloAx=angoloAx*(180/M_PI);
    
    angoloGx = angoloAx;
    
    fiAngx= angoloAx;
    
    Serial.print(angoloAx); Serial.print("\t");
      
    Serial.print(angoloGx); Serial.print("\t");
    /*---------------------------------------------------------------------- setto l'ora a zero --------------------------------------------*/
    
    DateTime.sync(time);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // 
        /*------------------------------------------------------------------- lettura ed elaborazione dati accelerometro -------------------*/
        angoloAx=atan((ax)/(sqrt(pow(ay,2)+pow(az,2))));
        
        /*------------------------------------------------------------------- conversione da radianti a gradi ------------------------------*/
        angoloAx=angoloAx*(180/M_PI);
        
        /*------------------------------------------------------------------- salvo i secondi correnti in una variabile---------------------*/
        nowTime = DateTime.now(); 
        
        /*------------------------------------------------------------------- elaborazione dati giroscopio ---------------------------------*/
        
          angoloGx = fiAngx+(gx/(131)*(1/25));
          
          oldTime=nowTime;
          
          /*------- risetto i secondi a 0 per non andare in overflow --------*/
          if(nowTime>=60)
          {
            DateTime.sync(time);
          }
        /*------------------------------------------------------------------- filtro complementare -----------------------------------------*/
        
        fiAngx = (0.75)*(angoloGx)+(0.25)*(angoloAx);
        
        /*------------------------------------------------------------------- print dei risultati ------------------------------------------*/
        Serial.print("a/g:\t");
        
        Serial.print(angoloAx); Serial.print("\t");
        
        Serial.print(angoloGx); Serial.print("\t");
        
        Serial.print(nowTime); Serial.print("\t");
        
        Serial.print(fiAngx); Serial.print("\n");
        
        /*------------------------------------------------------------------ Controllo PID -------------------------------------------------*/
        
        //proportional control
        error = filteredXAngle - setPoint; //because of our setPoint matches with 0 degrees, we can just use the filteredXAngle as our error
        P = kp * error;
    
        //integral control
        sumError = sumError + (error * samplingT);
        I = kI * sumError;
    
        //differential control
        difError = (error - prevError)/samplingT;
        D = kD * difError;
    
        prevError = error;
        
        /*------------------------------------------------------------------ if per circuito di prova --------------------------------------*/
        if(ax<-4000)
        {
          digitalWrite(10,HIGH);
        }
        else{
              if(ax>4000)
              {
                digitalWrite(12,HIGH);
              }
              else{
                    if(ay<-4000)
                    {
                      digitalWrite(9,HIGH);
                    }
                    else{
                          if(ay>4000)
                          {
                            digitalWrite(11,HIGH);
                          }
                          else{
                                digitalWrite(12,LOW);
                                digitalWrite(11,LOW);
                                digitalWrite(10,LOW);
                                digitalWrite(9,LOW);
                          }
                    }
              }
        }
    

   /* #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif*/

    // blink LED to indicate activity
    /*blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);*/
    
    
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

}
