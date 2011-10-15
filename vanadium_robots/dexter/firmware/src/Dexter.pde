/* 
  ArbotiX Firmware for ROS driver
  Copyright (c) 2008-2011 Vanadium Labs LLC.  All right reserved.
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/ 

/* Build Configuration */
//#define USE_BASE            // Enable support for a mobile base
//#define USE_BIG_MOTORS      // Enable Pololu 30A support
//#define USE_HW_SERVOS       // Enable only 2/8 servos, but using hardware control
#define USE_NUKE

/* Hardware Constructs */
#include <ax12.h>
#include <BioloidController.h>

#ifndef USE_NUKE
    BioloidController bioloid = BioloidController(1000000);
#endif

#include "ros.h"

#ifdef USE_HW_SERVOS
  #include <HServo.h>
  HServo servos[2];
#else 
  #include <Servo.h>
  Servo servos[10];
#endif

#ifdef USE_BASE
  #ifdef USE_BIG_MOTORS
    #include <BigMotors.h>
    BigMotors drive = BigMotors();
  #else
    #include <Motors2.h>
    Motors2 drive = Motors2();
  #endif
  #include <EncodersAB.h>
  #include "pid.h"
#endif

#ifdef USE_NUKE
  #include "nuke.h"
#endif

/* Register Storage */
unsigned char baud = 7;         // ?
unsigned char ret_level = 1;    // ?
unsigned char alarm_led = 0;    // ?
int temp;                       // ?
int servo_vals[10];             // in millis

/* Pose & Sequence Structures */
typedef struct{
  unsigned char pose;           // index of pose to transition to 
  int time;                     // time for transition
} sp_trans_t;
int poses[30][AX12_MAX_SERVOS]; // poses [index][servo_id-1]
sp_trans_t sequence[50];        // sequence
int seqPos;                     // step in current sequence

/* 
 * Setup Functions
 */
#if defined(AX_RX_SWITCHED)
void scan(){
  // do a search for devices on the RX bus, default to AX if not found
  int i;
  for(i=0;i<AX12_MAX_SERVOS;i++){
    dynamixel_bus_config[i] = 1;
    if(ax12GetRegister(i+1, AX_ID, 1) != (i+1)){
      dynamixel_bus_config[i] = 0;
    }
  }  
}
#endif
void setup(){
  // setup serial
  Serial.begin(115200);  

#ifdef USE_BASE  
  Encoders.Begin();
  setupPID();
#endif

#if defined(AX_RX_SWITCHED)
  delay(1000);
  scan();
#endif

#ifdef USE_NUKE
  setupNUKE();
#endif

  pinMode(0,OUTPUT);     // status LED
}

/*
 * Handle Write Requests to ArbotiX Registers 
 */
unsigned char handleWrite(){
  int addr  = params[0];  // address to write
  int bytes = length-3;   // # of bytes left to write
  int k = 1;              // index in parameters of value to write
  while(bytes > 0){
    if(addr < REG_BAUD_RATE){
      return INSTRUCTION_ERROR;
    }else if(addr == REG_BAUD_RATE){
      UBRR1L = params[k];      
    }else if(addr < REG_RESCAN){
      // write digital 
      int pin = addr - REG_DIGITAL;
    #ifdef SERVO_STIK
      pin = 31-pin;
    #endif
      if(params[k] & 0x02)    // high
        digitalWrite(pin, HIGH);
      else
        digitalWrite(pin, LOW);
      if(params[k] & 0x01)    // output
        pinMode(pin, OUTPUT);
      else
        pinMode(pin, INPUT);
    }else if(addr == REG_RESCAN){
#if defined(AX_RX_SWITCHED)
      scan();
#endif
    }else if(addr == REG_RETURN_LEVEL){
      ret_level = params[k];
    }else if(addr == REG_ALARM_LED){
      // TODO: 
    }else if(addr < REG_SERVO_BASE){
      return INSTRUCTION_ERROR; // error - analog are read only
    }else if(addr < REG_MOVING){
      // write servo
      int s = addr - REG_SERVO_BASE;
 #ifdef USE_HW_SERVO
      if( s >= 4 ){
 #else
      if( s >= 20){
 #endif 
        return INSTRUCTION_ERROR;
      }else{
        if( s%2 == 0 ){ // low byte
          s = s/2;
          servo_vals[s] = params[k];
        }else{          // high byte
          s = s/2;
          servo_vals[s] += (params[k]<<8);
          if(servo_vals[s] > 500 && servo_vals[s] < 2500){
            servos[s].writeMicroseconds(servo_vals[s]);
            if(!servos[s].attached())            
              servos[s].attach(s);
          }else if(servo_vals[s] == 0){
            servos[s].detach();
          }
        }
      }
    }else if(addr == REG_MOVING){
      return INSTRUCTION_ERROR;
      
#ifdef USE_BASE
    // motor pwms = 1 byte sign + 1 byte value
    }else if(addr == REG_LM_SIGN){          
      left_pwm = 1 + -2*params[k];
    }else if(addr == REG_LM_PWM){
      left_pwm = left_pwm * params[k];
      PIDmode = 0;
      drive.left(left_pwm);
      if(left_pwm != 0)
        moving = 1;
    }else if(addr == REG_RM_SIGN){
      right_pwm = 1 + -2*params[k];
    }else if(addr == REG_RM_PWM){
      right_pwm = right_pwm * params[k];
      PIDmode = 0;
      drive.right(right_pwm);
      if(right_pwm != 0)
        moving = 1;
        
    // motor speed for move_base (ticks/frame, 2 bytes, signed)
    }else if(addr == REG_LM_SPEED_L){       
      left_speed = params[k];
    }else if(addr == REG_LM_SPEED_H){
      left_speed += (params[k]<<8);
    }else if(addr == REG_RM_SPEED_L){
      right_speed = params[k];
    }else if(addr == REG_RM_SPEED_H){
      right_speed += (params[k]<<8); 
      //assumed to be all written at once
      if((left_speed == 0) && (right_speed == 0)){
        drive.set(0,0);
        ClearPID();
      }else{
        if((left.Velocity == 0) && (right.Velocity == 0)){
          PIDmode = 1; moving = 1;
          left.PrevEnc = Encoders.left;
          right.PrevEnc = Encoders.right;
        }
      }   
      left.Velocity = left_speed;
      right.Velocity = right_speed; 
      
    }else if(addr < REG_KP){
      // can't write encoders?  
      return INSTRUCTION_ERROR;
    }else if(addr == REG_KP){
      Kp = params[k];  
    }else if(addr == REG_KD){
      Kd = params[k];  
    }else if(addr == REG_KI){
      Ki = params[k];  
    }else if(addr == REG_KO){
      Ko = params[k];  
#endif

#ifdef USE_NUKE        
    // motor speed for move_nuke (mm/frame, 2 bytes, signed)
    }else if(addr == REG_X_SPEED_H){
      Xspeed = (params[k]<<8) + params[k-1];
    }else if(addr == REG_R_SPEED_H){
      Rspeed = ((float)(params[k]<<8) + params[k-1])/1000.0;
    }else if(addr == REG_Y_SPEED_H){
      Yspeed = (params[k]<<8) + params[k-1];
    // body rotations
    }else if(addr == REG_ROLL+1){
      bodyRotX = ((float)(params[k]<<8) + params[k-1])/1000.0;
    }else if(addr == REG_PITCH+1){
      bodyRotY = ((float)(params[k]<<8) + params[k-1])/1000.0;
    }else if(addr == REG_YAW+1){
      bodyRotZ = ((float)(params[k]<<8) + params[k-1])/1000.0;
    // robot height
    }else if(addr == REG_HEIGHT){
      for(int i=0; i<LEG_COUNT; i++)
        endpoints[i].z = params[k];
    // robot gait
    }else if(addr == REG_GAIT){
      gaitSelect(params[k]);
#endif

    }else{
      return INSTRUCTION_ERROR;
    }
    addr++;k++;bytes--;
  }
  return 0;
}

int handleRead(){
  int checksum = 0;
  int addr = params[0];
  int bytes = params[1];
  unsigned char v;
  while(bytes > 0){
    if(addr == REG_MODEL_NUMBER_L){ 
      v = 44;
    }else if(addr == REG_MODEL_NUMBER_H){
      v = 1;  // 300 
    }else if(addr == REG_VERSION){
      v = 0;
    }else if(addr == REG_ID){
      v = 253;
    }else if(addr == REG_BAUD_RATE){
      v = 34; // 56700
    }else if(addr < REG_RETURN_LEVEL){
      // send digital read
      if(addr == REG_DIGITAL){
        // 0->7
    #ifdef SERVO_STIK
        v = PINA;
    #else
        v = PINB;
    #endif
      }else if(addr == REG_DIGITAL+1){
        // 8-15
    #ifdef SERVO_STIK
        v = (PINB>>1);
    #else
        v = PIND;
    #endif        
      }else{
        // 16-23
        v = PIND;
      }
      
    }else if(addr == REG_RETURN_LEVEL){
      v = ret_level;
    }else if(addr == REG_ALARM_LED){
      // TODO
    }else if(addr < REG_SERVO_BASE){
      // send analog reading
      v = analogRead(addr-REG_ANA_BASE)>>2;
    }else if(addr < REG_MOVING){
      // send servo position
      v = 0;
      
#ifdef USE_BASE
    }else if(addr == REG_MOVING){
      v = moving;
    }else if(addr < REG_ENC_LEFT_L){
      v = 0;
    }else if(addr < REG_ENC_RIGHT_L){
      // send left encoder value
      int k = (addr - REG_ENC_LEFT_L)*8;
      v = ((unsigned long)Encoders.left>>k)%256;
    }else if(addr < REG_KP){
      // send right encoder values
      int k = (addr - REG_ENC_RIGHT_L)*8;
      v = ((unsigned long)Encoders.right>>k)%256;
    }else if(addr == REG_KP){
      v = Kp;
    }else if(addr == REG_KD){
      v = Kd;
    }else if(addr == REG_KI){
      v = Ki;
    }else if(addr == REG_KO){
      v = Ko;
#endif

#ifdef USE_NUKE
    }else if(addr == REG_MOVING){
      v = 0; //v = moving;
    }else if(addr < REG_ENC_X_L){
      v = 0;
    }else if(addr < REG_ENC_R_L){
      // send X encoder value
      int k = (addr - REG_ENC_X_L)*8;
      v = ((unsigned long)EncoderX>>k)%256;
    }else if(addr < REG_ENC_Y_L){
      // send R encoder values
      int k = (addr - REG_ENC_RIGHT_L)*8;
      v = ((unsigned long)(EncoderR*1000.0)>>k)%256;
    }else if(addr < REG_KP){
      // send Y encoder values
      int k = (addr - REG_ENC_RIGHT_L)*8;
      v = ((unsigned long)EncoderY>>k)%256;
#endif
      
    }else{
      v = 0;        
    }
    checksum += v;
    Serial.print(v, BYTE);
    addr++;bytes--;
  }
  return checksum;
}

int doPlaySeq(){
  seqPos = 0; int i;
  while(sequence[seqPos].pose != 0xff){
    int p = sequence[seqPos].pose;
    // are we HALT?
    if(Serial.read() == 'H') return 1;
    // load pose
    for(i=0; i<bioloid.poseSize; i++)
      bioloid.setNextPose(i+1,poses[p][i]); 
    bioloid.interpolateSetup(sequence[seqPos].time);
    while(bioloid.interpolating)
      bioloid.interpolateStep();
    // next transition
    seqPos++;
  }
  return 0;
}

/*
 * Send status packet
 */
void statusPacket(int id, int err){
  Serial.print(0xff,BYTE);
  Serial.print(0xff,BYTE);
  Serial.print(id,BYTE);
  Serial.print(2,BYTE);
  Serial.print(err,BYTE);
  Serial.print(255-((id+2+err)%256),BYTE);
}

/* 
 * decode packets: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix 
 */
void loop(){
  int i;
    
  // process messages
  while(Serial.available() > 0){
    // We need to 0xFF at start of packet
    if(mode == 0){         // start of new packet
      if(Serial.read() == 0xff){
        mode = 2;
        digitalWrite(0,HIGH-digitalRead(0));
      }
    //}else if(mode == 1){   // another start byte
    //    if(Serial.read() == 0xff)
    //        mode = 2;
    //    else
    //        mode = 0;
    }else if(mode == 2){   // next byte is index of servo
      id = Serial.read();    
      if(id != 0xff)
        mode = 3;
    }else if(mode == 3){   // next byte is length
      length = Serial.read();
      checksum = id + length;
      mode = 4;
    }else if(mode == 4){   // next byte is instruction
      ins = Serial.read();
      checksum += ins;
      index = 0;
      mode = 5;
    }else if(mode == 5){   // read data in 
      params[index] = Serial.read();
      checksum += (int) params[index];
      index++;
      if(index + 1 == length){  // we've read params & checksum
        mode = 0;
        if((checksum%256) != 255){ 
          // return an error packet: FF FF id Len Err=bad checksum, params=None check
          statusPacket(id,CHECKSUM_ERROR);
        }else if(id == 253){  // ID = 253, ArbotiX instruction
          switch(ins){     
            case AX_WRITE_DATA:
              // send return packet
              statusPacket(id,handleWrite());
              break;
             
            case AX_READ_DATA:
              checksum = id + params[1] + 2;                            
              Serial.print(0xff,BYTE);
              Serial.print(0xff,BYTE);
              Serial.print(id,BYTE);
              Serial.print(2+params[1],BYTE);
              Serial.print(0,BYTE);
              // send actual data
              checksum += handleRead();
              Serial.print(255-((checksum)%256),BYTE);
              break;
             
            case ARB_SIZE_POSE:                   // Pose Size = 7, followed by single param: size of pose
              statusPacket(id,0);
              bioloid.poseSize = params[0];
              bioloid.readPose();    
              break;
             
            case ARB_LOAD_POSE:                   // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size)
              statusPacket(id,0);
              for(i=0; i<bioloid.poseSize; i++)
                poses[params[0]][i] = params[(2*i)+1]+(params[(2*i)+2]<<8); 
              break;
             
            case ARB_LOAD_SEQ:                    // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size) 
              statusPacket(id,0);
              for(i=0;i<(length-2)/3;i++){
                sequence[i].pose = params[(i*3)];
                sequence[i].time = params[(i*3)+1] + (params[(i*3)+2]<<8);
              }
              break;
             
            case ARB_PLAY_SEQ:                   // Play Seq = A, no params   
              statusPacket(id,0);
              doPlaySeq();
              break;
             
            case ARB_LOOP_SEQ:                   // Play Seq until we recieve a 'H'alt
              statusPacket(id,0);
              while(doPlaySeq() > 0);
              break;

#ifdef USE_BASE   
            case ARB_TEST:
              statusPacket(id,0);
              doTest();
              break;
#endif
          }
        }else if(id == 0xFE){
          // sync read or write
          if(ins == ARB_SYNC_READ){
            int start = params[0];    // address to read in control table
            int bytes = params[1];    // # of bytes to read from each servo
            int k = 2;
            checksum = id + (bytes*(length-4)) + 2;                            
            Serial.print(0xff,BYTE);
            Serial.print(0xff,BYTE);
            Serial.print(id,BYTE);
            Serial.print(2+(bytes*(length-4)),BYTE);
            Serial.print(0,BYTE);     // error code
            // send actual data
            for(k=2; k<length-2; k++){
              ax12GetRegister(params[k], start, bytes);
              for(i=0;i<bytes;i++){
                checksum += ax_rx_buffer[5+i];
                Serial.print(ax_rx_buffer[5+i],BYTE);
              }
            }
            Serial.print(255-((checksum)%256),BYTE);
          }else{    
            // TODO: sync write pass thru
            int k;
            setTXall();
            ax12write(0xff);
            ax12write(0xff);
            ax12write(id);
            ax12write(length);
            ax12write(ins);
            for(k=0; k<length; k++)
                ax12write(params[k]);
            // no return
          }       
        }else{ // ID != 253, pass thru 
          switch(ins){
            // TODO: streamline this
            case AX_READ_DATA:
              ax12GetRegister(id, params[0], params[1]);
              // return a packet: FF FF id Len Err params check
              if(ax_rx_buffer[3] > 0){
                for(i=0;i<ax_rx_buffer[3]+4;i++)
                  Serial.print(ax_rx_buffer[i],BYTE);
              }
              ax_rx_buffer[3] = 0;
              break;
             
            case AX_WRITE_DATA:
              if(length == 4){
                ax12SetRegister(id, params[0], params[1]);
              }else{
                int x = params[1] + (params[2]<<8);
                ax12SetRegister2(id, params[0], x);
              }
              statusPacket(id,0);
              break;
             
          }
        }
      }
    } // end mode == 5
  } // end while(available)

#ifdef USE_NUKE
  updateNUKE();
#endif

  // update joints
  bioloid.interpolateStep();
 
#ifdef USE_BASE
  // update pid
  updatePID();
#endif

}

