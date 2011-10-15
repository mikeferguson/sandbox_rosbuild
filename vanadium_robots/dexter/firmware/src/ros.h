/* 
  Common Definitions for ROS driver ArbotiX Firmware
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

/* ArbotiX (id:253) Instruction Definitions */
#define ARB_SIZE_POSE       7    // pose size: a single param for size of pose
#define ARB_LOAD_POSE       8    // load pose: index, then pose positions (# of params = 2*pose_size)
#define ARB_LOAD_SEQ        9    // seq size: a single param for the size of the seq
#define ARB_PLAY_SEQ        10   // load seq: index/times (# of params = 3*seq_size)
#define ARB_LOOP_SEQ        11   // play seq: no params
#define ARB_TEST            25   // hardware test: no params
#define ARB_SYNC_READ       0x84

/* ArbotiX (id:253) Register Table Definitions */
#define REG_MODEL_NUMBER_L  0
#define REG_MODEL_NUMBER_H  1
#define REG_VERSION         2
#define REG_ID              3
#define REG_BAUD_RATE       4
#define REG_DIGITAL         5   // First block of digital pins to read
                                // base + index, bit 1 = value (0,1), bit 0 = direction (0,1)
#define REG_RESCAN          15
#define REG_RETURN_LEVEL    16
#define REG_ALARM_LED       17
#define REG_ANA_BASE        18  // First Analog Port
#define REG_SERVO_BASE      26  // Up to 10 servos, each uses 2 bytes (L, then H), pulse width (0, 1000-2000ms)
#define REG_MOVING          46
#define REG_LM_SIGN         47  // Raw motor pwm (-255 to 255), 1 byte sign + 1 byte speed per side
#define REG_LM_PWM          48
#define REG_RM_SIGN         49
#define REG_RM_PWM          50

#define REG_LM_SPEED_L      51  // Motor Speed (ticks/sec, 2 bytes, signed)
#define REG_LM_SPEED_H      52
#define REG_RM_SPEED_L      53
#define REG_RM_SPEED_H      54

#define REG_X_SPEED_L       51  // OR Nuke Speed (mm/s, 2 bytes, signed) 
#define REG_X_SPEED_H       52
#define REG_R_SPEED_L       53
#define REG_R_SPEED_H       54
#define REG_Y_SPEED_L       55
#define REG_Y_SPEED_H       56

#define REG_ENC_LEFT_L      57  // Current Encoder Values (ticks, 4 bytes, signed)    
#define REG_ENC_RIGHT_L     61

#define REG_ENC_X_L         57  // OR Nuke Encoder Values (mm, 4 bytes, signed)
#define REG_ENC_R_L         61
#define REG_ENC_Y_L         65

#define REG_KP              69  // PID parameters
#define REG_KD              70
#define REG_KI              71
#define REG_KO              72

#define REG_ROLL            69  // OR Body parameters
#define REG_PITCH           71
#define REG_YAW             73
#define REG_HEIGHT          75
#define REG_GAIT            77

/* Packet Decoding */
int mode = 0;                   // where we are in the frame

unsigned char id = 0;           // id of this frame
unsigned char length = 0;       // length of this frame
unsigned char ins = 0;          // instruction of this frame

unsigned char params[143];      // parameters (match RX-64 buffer size)
unsigned char index = 0;        // index in param buffer

int checksum;                   // checksum

#define INSTRUCTION_ERROR   0x40
#define CHECKSUM_ERROR      0x10

