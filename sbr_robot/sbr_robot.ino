#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "pid.h"
#include <avr/interrupt.h>

const int ledPin = 13, encL1 = 3, encL2 = 4, encR1 = 11, encR2 = 12;
#define GETENC_L1 (PIND & (1 << PD3));//D3
#define GETENC_L2 (PIND & (1 << PD4));//D4
#define GETENC_R1 (PINB & (1 << PB3));//D11
#define GETENC_R2 (PINB & (1 << PD4));//D12
bool blinkState = false;

// AD0 low = 0x68, AD0 high = 0x69
MPU6050 mpu(0x68);
bool dmpReady = false;
uint8_t mpuIntStatus, devStatus;
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }
/*  DEBUG MODES
 * 1: yaw pitch roll MPU6050 output
 * 2: controller values
 * 3: pitch_pid values
 * 4: encoder values
 * 5: vel_pid values
 */
#define DEBUG_MODE 0

// left
int enR = 5;
int inR2 = 7;
int inR1 = 6;
// right
int enL = 10;
int inL2 = 9;
int inL1 = 8;

void driveL(int n) {
    if(n > 255) n = 255;
    if(n < -255) n = -255;
    if (n < 0) {
        digitalWrite(inL1, 0);
        digitalWrite(inL2, 1);
    } else {
        digitalWrite(inL1, 1);
        digitalWrite(inL2, 0);
    }
    analogWrite(enL, abs(n));
}
void driveR(int n) {
    if(n > 255) n = 255;
    if(n < -255) n = -255;
    if (n < 0) {
        digitalWrite(inR1, 1);
        digitalWrite(inR2, 0);
    } else {
        digitalWrite(inR1, 0);
        digitalWrite(inR2, 1);
    }
    analogWrite(enR, abs(n));
}
PidVars pitch_pid = pidDef, vel_pid = pidDef;
/*
 *  TUNING OPTIONS
 * 0: none
 * 1: pitch
 * 2: vel
 */
int tuning = 0;
char s[60];
volatile unsigned char L1prev = 0, R1prev = 0;
volatile long leftEnc = 0, rightEnc = 0;
//interupt service routines:
//D8 to D13
//right
ISR(PCINT0_vect) {
  int R1val = GETENC_R1;
  int R2val = GETENC_R2;
  //falling edge
  if(!R1val && R1prev) {
    if(R2val) rightEnc--;
    else rightEnc++;
  }
  //rising edge
  else {
    if(R2val) rightEnc++;
    else rightEnc--;
  }
  R1prev = R1val;
}
//D0 to D7
//left
ISR(PCINT2_vect) {
  int L1val = GETENC_L1;
  int L2val = GETENC_L2;
  //falling edge
  if(!L1val && L1prev) {
    if(L2val) leftEnc++;
    else leftEnc--;
  }
  //rising edge
  else {
    if(L2val) leftEnc--;
    else leftEnc++;
  }
  L1prev = L1val;
}

void setup() {
    //////// CONFIGURE ENCODER INTERRUPTS ////////
    //disable interrupts
    cli();
    //disable existing flags from PCICR (Pin Change Control Register)
    PCICR &= 0b00000000;
    //set to Pin Change Interrupt 0 and 2
    PCICR |= 0b00000101;
    //enable them
    PCMSK0 |= 0b00011000; // D11, D12 PCINT3,4
    PCMSK2 |= 0b00011000; // D3, D4   PCINT19,20
    //reenable interrupts
    sei();
  
    Wire.begin();
    Serial.begin(115200);//bluetooth
    TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(1573);
    mpu.setYGyroOffset(4230);
    mpu.setZGyroOffset(-870);
    mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed. code "));
        Serial.print(devStatus);
    }
    pinMode(ledPin, OUTPUT);
    pinMode(enL, OUTPUT);
    pinMode(enR, OUTPUT);
    pinMode(inR1, OUTPUT);
    pinMode(inR2, OUTPUT);
    pinMode(inL1, OUTPUT);
    pinMode(inL2, OUTPUT);
    pinMode(encL1, INPUT);
    pinMode(encL2, INPUT);
    pinMode(encR1, INPUT);
    pinMode(encR2, INPUT);
    //agressive: kp=142, ki=1.035, kd=2830
    //medium: kp=99, ki=.4433, kd=2328
    //mild: kp=81, ki=.1507, kd=962
    pitch_pid.kp = 99;
    pitch_pid.ki = 0.4433;
    pitch_pid.kd = 2328;
    pitch_pid.maxIntegral = 255;
    pitch_pid.unwind = 0;
    pitch_pid.DERIVATIVE_ACTIVE_ZONE = 999999;
    pitch_pid.INTEGRAL_ACTIVE_ZONE = 999999;
    pitch_pid.sensVal = 0;
    pitch_pid.target = 0;
    vel_pid.kp = 2.03;
    vel_pid.ki = 0.0054;
    vel_pid.kd = 0.0;
    vel_pid.maxIntegral = 255;
    vel_pid.unwind = 0;
    vel_pid.DERIVATIVE_ACTIVE_ZONE = 999999;
    vel_pid.INTEGRAL_ACTIVE_ZONE = 999999;
    vel_pid.sensVal = 0;
    vel_pid.target = 0;
    
    if(DEBUG_MODE) return;
    
    for(int i = 0; i < 16; i++) {
      delay(1400);
      strcpy(s, "");
      for(int j = 0; j < 16; j++) {
        if(j <= i) {
          strcat(s, "=");
        } else {
          strcat(s, "-");
        }
      }
      Serial.println(s);
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState);
    }
}
void printEnc() {
    Serial.print("L:\t");
    Serial.print(leftEnc);
    Serial.print("\tR:\t");
    Serial.println(rightEnc);
}
unsigned long prevT = 0;
double prevDisp = 0;
int8_t jxVal = 0, jyVal = 0;
uint8_t butC = 0, butZ = 0;
double alpha = 0.88;
void loop() {
    if (!dmpReady){
      Serial.println("dmp not ready: exiting main loop");
      return;
    }
    if(Serial.available() > 11) {
      Serial.println("Serial buffer too big");
      while(Serial.available())Serial.read(); 
    }
    if(DEBUG_MODE == 4) printEnc();
    if (Serial.available() == 11) {
        jxVal = Serial.read();
        jyVal = Serial.read();
        butC = Serial.read();
        butZ = Serial.read();
        if(tuning == 1) {
          pitch_pid.kp = Serial.read() + Serial.read()/100.0;
          pitch_pid.ki = Serial.read() + (Serial.read() | (Serial.read() << 8))/10000.0;
          pitch_pid.kd = Serial.read() | (Serial.read() << 8);
        } else if(tuning == 2) {
          vel_pid.kp = Serial.read() + Serial.read()/100.0;
          vel_pid.ki = Serial.read() + (Serial.read() | (Serial.read() << 8))/10000.0;
          vel_pid.kd = Serial.read() | (Serial.read() << 8);
        } else {
          //ignore pid serial data
          for(int i = 0; i < 7; i++) Serial.read();
        }
        if(DEBUG_MODE == 2) {
          PidVars pid = pidDef;
          if(tuning == 1) pid = pitch_pid;
          if(tuning == 2) pid = vel_pid;
          Serial.print("kp: ");
          Serial.print((int)pid.kp);
          Serial.print(".");
          Serial.print((int)((pid.kp - (int)pid.kp) * 100));
          Serial.print("\tki: ");
          Serial.print((int)pid.ki);
          Serial.print(".");
          Serial.print((int)((pid.ki - (int)pid.ki)*10000));
          Serial.print("\tkd: ");
          Serial.print((int)pid.kd);/*
          Serial.print("\tjx: ");
          Serial.print(jxVal);
          Serial.print("\tjy: ");
          Serial.print(jyVal);
          Serial.print("\tbc: ");
          Serial.print(butC);
          Serial.print("\tbz: ");
          Serial.print(butZ);*/
          Serial.println();
        }
        if (jxVal > 90) jxVal = 90;
        if (jxVal < -90) jxVal = -90;
        if (jyVal > 90) jyVal = 90;
        if (jyVal < -90) jyVal = -90;
    }
    // wait for MPU interrupt or extra packet(s) available
    unsigned long start = millis();
    while (!mpuInterrupt && fifoCount < packetSize) {
      if(millis() - start > 2000) {
        Serial.println("ERROR: no interupt recieved or not enough data in packet (stuck waiting)");
      }
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        unsigned long start = millis();
        while (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
          if(millis() - start > 2000) {
            Serial.println("ERROR: not enough data in packet (stuck waiting for data)");
          }
        }
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        if(DEBUG_MODE == 1) {
          Serial.print("ypr\t");
          Serial.print(ypr[0] * 180 / M_PI);
          Serial.print("\t");
          Serial.print(ypr[1] * 180 / M_PI);
          Serial.print("\t");
          Serial.println(ypr[2] * 180 / M_PI);
        }
        double disp = (leftEnc + rightEnc) / 2.0;
        vel_pid.sensVal = (disp - prevDisp) / (millis() - prevT);
        prevT = millis();
        prevDisp = disp;
        vel_pid.target = jyVal * (1.4 / 90.0);
        
        pitch_pid.sensVal = -(ypr[1]*180.0/M_PI - 4.1);
        if(tuning == 1) {
          pitch_pid.target = -jyVal/15.0;//-3.95, -4.6
        } else {
          //calculate moving average of the output angle
          pitch_pid.target = pitch_pid.target*alpha + (-updatePID(&vel_pid))*(1-alpha);//-jyVal/15.0;//-3.95, -4.6
          double maxAngle = 25.0;
          if(pitch_pid.target > maxAngle) pitch_pid.target = maxAngle;
          if(pitch_pid.target < -maxAngle) pitch_pid.target = -maxAngle;
          
          Serial.print("\ts ");
          Serial.print(vel_pid.sensVal);
          Serial.print("\tt ");
          Serial.println(vel_pid.target);
        }
        
        double power = updatePID(&pitch_pid);
        double turningMag = jxVal / (4*fabs(vel_pid.sensVal) + 0.5);
        if(fabs(pitch_pid.sensVal) > 3.0) {
          turningMag = 0;
        }
        if(!DEBUG_MODE && fabs(pitch_pid.target - pitch_pid.sensVal) < 45) {
          driveL(power + turningMag);
          driveR(power - turningMag);
        } else {
          driveL(0);
          driveR(0);
        }
        if(DEBUG_MODE == 3) {
          Serial.print("\tsv: ");
          Serial.print(pitch_pid.sensVal);
          Serial.print("\terr: ");
          Serial.print(pitch_pid.target - pitch_pid.sensVal);
          Serial.print("\tpower: ");
          Serial.println(power);
        }
        if(DEBUG_MODE == 5) {
          Serial.print("\tsv: ");
          Serial.print(vel_pid.sensVal);
          Serial.print("\terr: ");
          Serial.println(vel_pid.target - vel_pid.sensVal);
        }
        blinkState = !blinkState;
        digitalWrite(ledPin, blinkState);
    }
}
