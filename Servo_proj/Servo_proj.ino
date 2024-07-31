#include <Adafruit_PWMServoDriver.h>

/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define    SERVOMIN           60                //100        // 100 this is the 'minimum' pulse length count (out of 4096)
#define    SERVOMAX           300              // 325                // 600 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
//uint8_t     servonum = 0;  
////-------serial----------------//////
#define   NUM         1800

uint8_t   pin_num0  =  2;
uint8_t   pin_num1  =  4;
uint8_t   pin_num2  =  7;
uint8_t   pin_num3  =  8;
uint8_t    finger_flag  =   0x0; 
uint8_t    servonum     =    8;
uint8_t    frame_count  =    0;

void setup() {
  Serial.begin(9600);
//  Serial.begin(115200);      // baund rate  
  Serial.println("16 channel Servo test!");
  pwm.begin();   
  pwm.setPWMFreq(60);      //  Analog servos run at ~60 Hz updates
  /////----modeset
  pinMode(pin_num0 ,  INPUT);
  pinMode(pin_num1 ,  INPUT);
  pinMode(pin_num2 ,  INPUT);
  pinMode(pin_num3 ,  INPUT);
}


void clear_buffer()  {
    while (Serial.read() >=0);             //  串口接收到数据
  //  clear serial buffer
     Serial.println("clear buffer");
}


// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void    setServoPulse(uint8_t n, double pulse) {
  double    pulselength;
  
  pulselength    =  1000000;           //   1,000,000 us per second
  pulselength   /= 60;               //   60 Hz
  Serial.print(pulselength);
  Serial.println(" us per period"); 
  pulselength  /=  4096;            //    12 bits of resolution
  Serial.print(pulselength); 
  Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}


void loop() {
  // Drive each servo one at a time

Serial.println(servonum); ////

//-----------------------Serial mode---------------------------------///  
if (Serial.available() > 0)             //  串口接收到数据
  {
      Serial.println("Serial   Setup!");    
   int  finger_num  =   Serial.read();        //  获取串口接收到的数据--        
      if (finger_num  ==   1)                 
      {
        finger_flag      =      0x1;
        Serial.println(" Recv Data1 OK! ");       
      }   
       else if (finger_num == 2)                     
      {
                finger_flag    =   0x2;
        Serial.println(" Recv Data2 OK! ");      
      }   
      else if (finger_num == 3)                     
      {
               finger_flag      =     0x3;
        Serial.println(" Recv Data3 OK! ");      
      }   
        else if (finger_num == 4)                   
      {
               finger_flag     =     0x4;        
        Serial.println(" Recv Data4 OK! ");      
		
      }   
        else if (finger_num == 5)                     
			
      {
               finger_flag     =     0x5;            
        Serial.println(" Recv Data5 OK! ");      
		
      }         
      else if  (finger_num == 'Good')               
		  
      {
               finger_flag     =     0x6;          
        Serial.println(" Recv GOOD! ");      
		
      }   
      else {
         Serial.println(finger_num);      
		 
      }
//    frame_count++; 
     Serial.println( finger_flag,  1 );  

  if (finger_flag     ==   0x1  ){   
        for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
        pwm.setPWM(2, 0, pulselen);
        pwm.setPWM(3, 0, pulselen);
        }    
        delay(500);
        for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(2, 0, pulselen);
        pwm.setPWM(3, 0, pulselen);
        }    
        frame_count++;  
   // 清除串口缓冲区
     clear_buffer(  );
//     Serial.flush(  );     
  }
  if (finger_flag     ==   0x2   ){     
        for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
        pwm.setPWM(2, 0, pulselen);
        pwm.setPWM(3, 0, pulselen);
        pwm.setPWM(4, 0, pulselen);
        pwm.setPWM(5, 0, pulselen);    
        }    
        delay(500);
        for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(2, 0, pulselen);
        pwm.setPWM(3, 0, pulselen);
        pwm.setPWM(4, 0, pulselen);
        pwm.setPWM(5, 0, pulselen);    
        }    
               frame_count++;             
   // 清除串口缓冲区
     clear_buffer(  );
//     Serial.flush(  );               
  } 
  if (finger_flag    ==   0x3 ){     
        for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
        pwm.setPWM(2, 0, pulselen);
        pwm.setPWM(3, 0, pulselen);
        pwm.setPWM(4, 0, pulselen);
        pwm.setPWM(5, 0, pulselen);    
        pwm.setPWM(6, 0, pulselen);
        pwm.setPWM(7, 0, pulselen);  
        }    
        delay(500);
        for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(2, 0, pulselen);
        pwm.setPWM(3, 0, pulselen);
        pwm.setPWM(4, 0, pulselen);
        pwm.setPWM(5, 0, pulselen);    
        pwm.setPWM(6, 0, pulselen);
        pwm.setPWM(7, 0, pulselen);  
        }    
               frame_count++;   
      // 清除串口缓冲区
         clear_buffer(  );
//          Serial.flush(  );              
  } 
  if (finger_flag  ==   0x4   ){     
        for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
        pwm.setPWM(2, 0, pulselen);
        pwm.setPWM(3, 0, pulselen);
        pwm.setPWM(4, 0, pulselen);
        pwm.setPWM(5, 0, pulselen);    
        pwm.setPWM(6, 0, pulselen);
        pwm.setPWM(7, 0, pulselen); 
        pwm.setPWM(8, 0, pulselen);
        pwm.setPWM(9, 0, pulselen);     
        }    
        delay(500);
        for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(2, 0, pulselen);
        pwm.setPWM(3, 0, pulselen);
        pwm.setPWM(4, 0, pulselen);
        pwm.setPWM(5, 0, pulselen);    
        pwm.setPWM(6, 0, pulselen);
        pwm.setPWM(7, 0, pulselen);  
        pwm.setPWM(8, 0, pulselen);
        pwm.setPWM(9, 0, pulselen);      
        }  
                frame_count++;                        
   // 清除串口缓冲区
     clear_buffer(  );
//     Serial.flush(  );  
} 
   
   if (finger_flag     ==   0x5  ){   
          for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
          pwm.setPWM(0, 0, pulselen);
          pwm.setPWM(1, 0, pulselen);      
          pwm.setPWM(2, 0, pulselen);
          pwm.setPWM(3, 0, pulselen);
          pwm.setPWM(4, 0, pulselen);
          pwm.setPWM(5, 0, pulselen);    
          pwm.setPWM(6, 0, pulselen);
          pwm.setPWM(7, 0, pulselen); 
          pwm.setPWM(8, 0, pulselen);
          pwm.setPWM(9, 0, pulselen);    
          }    
          delay(500);
          for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
          pwm.setPWM(0, 0, pulselen);
          pwm.setPWM(1, 0, pulselen);      
          pwm.setPWM(2, 0, pulselen);
          pwm.setPWM(3, 0, pulselen);
          pwm.setPWM(4, 0, pulselen);
          pwm.setPWM(5, 0, pulselen);    
          pwm.setPWM(6, 0, pulselen);
          pwm.setPWM(7, 0, pulselen);  
          pwm.setPWM(8, 0, pulselen);
          pwm.setPWM(9, 0, pulselen);      
          }  
                  frame_count++;                                 
  }
         
else{
         for (uint16_t pulselen = SERVOMIN;   pulselen < SERVOMAX;  pulselen++) {
           pwm.setPWM(0, 0, pulselen);
           pwm.setPWM(1, 0, pulselen);
           pwm.setPWM(2, 0, pulselen);
           pwm.setPWM(3, 0, pulselen);
           pwm.setPWM(4, 0, pulselen);
           pwm.setPWM(5, 0, pulselen);
           pwm.setPWM(6, 0, pulselen);
           pwm.setPWM(7, 0, pulselen);
           pwm.setPWM(8, 0, pulselen);
          pwm.setPWM(9, 0, pulselen);
         }
         delay(500);
         for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
           pwm.setPWM(0, 0, pulselen);
           pwm.setPWM(1, 0, pulselen);
           pwm.setPWM(2, 0, pulselen);
           pwm.setPWM(3, 0, pulselen);
           pwm.setPWM(4, 0, pulselen);
           pwm.setPWM(5, 0, pulselen);
           pwm.setPWM(6, 0, pulselen);
           pwm.setPWM(7, 0, pulselen);
           pwm.setPWM(8, 0, pulselen);
           pwm.setPWM(9, 0, pulselen);
       } 
           Serial.println(" Waiting Time! ");      ///   串口监视器里看打印出的东西
 } 
             finger_flag                   =               0x0; 
       Serial.println(finger_flag); 
     //      frame_count                   =                0;      
               delay(500);     
      // 清除串口缓冲区
               clear_buffer(  );
               Serial.flush(  );
  }  

}
