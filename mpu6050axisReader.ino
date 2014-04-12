#include <MPU6050.h>

#include <I2Cdev.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "math.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h

#include "Wire.h"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;



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
    int i=0;
    double angoloGx=0;
    double angoloGy=0;
    double angoloGz=0;
    double angoloAx=0;
    double angoloAy=0;
    double angoloAz=0;
    double lastAx=0;
    double lastAy=0;
    double lastAz=0;
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
    Serial.print("       ax     |ay     |az     |gx     |gy     |gz     |\n");
    /*-------------------------------------------------------------------------- imposto l'angolo inziale per il giroscopio -----------------*/
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    angoloAx=atan((ax)/(sqrt(pow(ay,2)+pow(az,2))));
    angoloAy=atan((ay)/(sqrt(pow(ax,2)+pow(az,2))));
    angoloAz=atan((az)/(sqrt(pow(ax,2)+pow(ay,2))));
    
    angoloAx=angoloAx*(180/M_PI);
    angoloAy=angoloAy*(180/M_PI);
    angoloAz=angoloAz*(180/M_PI);
    
    angoloGx = angoloAx;
    angoloGy = angoloAy;
    angoloGz = angoloAz; 
    
    Serial.print(angoloAx); Serial.print("\t");
    Serial.print(angoloAy); Serial.print("\t");
    Serial.print(angoloAz); Serial.print("\t");
      
    Serial.print(angoloGx); Serial.print("\t");
    Serial.print(angoloGy); Serial.print("\t");
    Serial.print(angoloGz); Serial.print("\n");
    /*---------------------------------------------------------------------------------------------------------------------------------------*/
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // 
        /*------------------------------------------------------------------- lettura ed elaborazione dati accelerometro -------------------*/
        angoloAx=atan((ax)/(sqrt(pow(ay,2)+pow(az,2))));
        angoloAy=atan((ay)/(sqrt(pow(ax,2)+pow(az,2))));
        angoloAz=atan((az)/(sqrt(pow(ax,2)+pow(ay,2))));
        
        /*------------------------------------------------------------------- conversione da radianti a gradi ------------------------------*/
        angoloAx=angoloAx*(180/M_PI);
        angoloAy=angoloAy*(180/M_PI);
        angoloAz=angoloAz*(180/M_PI);
        
        /*------------------------------------------------------------------- elaborazione dati giroscopio ---------------------------------*/
        if(i==1050)
        {
          angoloGx = lastAx + gx/(131);
          angoloGy = lastAy + gy/(131);
          angoloGz = lastAz + gz/(131);
          
          lastAx = angoloAx ;
          lastAy = angoloAy ;
          lastAz = angoloAz ;
          /*------------------------------TODO-------------------------------*/
          
            /*   Inserire al posto di angoloAx, angoloAy, angoloAz, */ 
            /*   gli angoli filtrati per una maggiore stabilità     */
            
          /*-----------------------------------------------------------------*/
          
          i=0;
        }
        i++;
        
        /*------------------------------------------------------------------- print dei risultati ------------------------------------------*/
        Serial.print("a/g:\t");
        
        Serial.print(angoloAx); Serial.print("\t");
        Serial.print(angoloAy); Serial.print("\t");
        Serial.print(angoloAz); Serial.print("\t");
        
        Serial.print(angoloGx); Serial.print("\t");
        Serial.print(angoloGy); Serial.print("\t");
        Serial.print(angoloGz); Serial.print("\n");
        
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
