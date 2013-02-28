/******************************************************************************
 * Bridge.c - A Mini-Servo Connector for the Bioloid Bus
 * 
 * Copyright (c) 2010, Michael E. Ferguson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * - Neither the name of the copyright holders nor the names of
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/******************************************************************************
 * Based on an 16MHz ATMEGA168
 * 
 * Goal =    0 =   0 degrees = 500us
 * Goal = 1023 = 180 degrees = 2500us
 *****************************************************************************/

#include <avr/interrupt.h>
#include "bridge.h"

// Globals
int device_id; 
volatile signed char channel;
servo_t servos[8];              // Servo 0-5 = PC5->0, 
                                // Servo 6 = PB5, Servo 7 = PB4
unsigned char shared_table[] =    
{12, 0, 0, 1, 1, 250, 0, 0,  
 255, 3, 0, 85, 60, 190, 255, 3, 
 2, 4, 4, 0, 0, 0, 0, 0, 0, 
 0, 0, 0, 0, 0, 32, 32, 0, 2, 
 0, 0, 255, 3, 0, 2, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 
 32, 0};

/******************************************************************************
 * Buffer and serial interactions
 */
unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile int rx_head;
volatile int rx_tail;

/** Read aspects* */
ISR(USART_RX_vect){
    rx_buffer[rx_head] = UDR0;
    rx_head = (rx_head+1); //%RX_BUFFER_SIZE;
}
int read(){   
    if(rx_tail == rx_head)
        return -1;
    int x = (int) rx_buffer[rx_tail];
    rx_tail = (rx_tail+1); //%RX_BUFFER_SIZE;
    return x;    
}
/** Sends a character out the serial port. */
void write(unsigned char data){
    while (bit_is_clear(UCSR0A, UDRE0));
    UDR0 = data;
}

/* Set the baud rate, this depends on us having a 16Mhz clock */
int setBaud(int baud){
    UCSR0A |= (1<<U2X0);
    if((baud == 1) || (baud == 3) || (baud==4) || (baud==7) ||
       (baud == 9) || (baud == 16) || (baud==50) || (baud ==103) || (baud == 207)){
        shared_table[AX_BAUD_RATE] = baud;  // 50 is our special 38.4k
        UBRR0L = baud;  
        return 1;
    }else{
        return -1;
    }
}
/** helper functions to emulate half-duplex */
void setTX(){
    UCSR0B &= ~(1<<RXCIE0);
    UCSR0B &= ~(1<<RXEN0);    
    UCSR0B |= (1<<TXEN0);
}
void setRX(){ 
    //rx_head = rx_tail = 0;
    UCSR0B &= ~(1<<TXEN0);
    UCSR0B |= (1<<RXEN0);   
    UCSR0B |= (1<<RXCIE0);
}

/******************************************************************************
 * Servo handling
 */
//ISR(TIMER1_COMPA_vect){
void doServo(){
    if( channel < 0 )
        TCNT1 = 0; // channel set to -1 indicated that refresh interval completed so reset the timer 
    else{
        if(servos[channel].torque > 0){
            // pulse channel low
            if(channel < 6)
                PORTC &= (0xff - (1<<(5-channel)));
            else if(channel == 6)
                PORTB &= 0xef;  // pb4
            else
                PORTB &= 0xf7;  // pb3
        }
    }

    channel++;    // increment to the next channel
    if( channel < 8 ){  
        if( servos[channel].torque > 0){
            OCR1A = TCNT1 + servos[channel].output;
            // set high
            if(channel < 6)
                PORTC |= 1<<(5-channel);
            else if(channel == 6)
                PORTB |= 0x10;  // pb4
            else
                PORTB |= 0x08;  // pb3
        }else{
            OCR1A = TCNT1 + 1;
        }
    }else{
        // finished all channels -- wait for refresh period before starting over
        if( (unsigned)TCNT1 < SERVO_FRAME + 4)   
            OCR1A = (unsigned int) SERVO_FRAME;
        else
            OCR1A = TCNT1 + 4;
        channel = -1;
    }        
}

/******************************************************************************
 * Packet Level
 */

void statusPacket(int id, int error, unsigned char * params, int p_cnt){
    int chk = id+2+p_cnt+error;
    // TODO: delay enough time
    delayus(25 /*RETURN_DELAY*/);
    // send data
    setTX();    
    write(0xFF);
    write(0xFF);
    write(id);
    write(2+p_cnt);
    write(error);
    while(p_cnt > 0){
        write(*params);
        chk += *params;
        params++;
        p_cnt--;
    } 
    write(~((unsigned char)chk%256));
    setRX();
}

/******************************************************************************
 * Packet Instructions
 */

/** Packet Parameters **/
int p_id; int p_len; int p_ins;
int p_params[50];
int p_idx; 
int p_chk;

unsigned char p_output[50];

void doRead(unsigned char id){
    if(RETURN_LEVEL == RETURN_NONE) return;
    shared_table[AX_ID]=id;
    shared_table[AX_TORQUE_ENABLE]=servos[id-device_id].torque;
    shared_table[AX_LED]= servos[id-device_id].led;
    shared_table[AX_GOAL_POSITION_L]=servos[id-device_id].goal_l;
    shared_table[AX_GOAL_POSITION_H]=servos[id-device_id].goal_h;
    shared_table[AX_PRESENT_POSITION_L]=servos[id-device_id].goal_l;
    shared_table[AX_PRESENT_POSITION_H]=servos[id-device_id].goal_h;
    statusPacket(id,0,shared_table+p_params[0],p_params[1]);
}

void doWrite(unsigned char id, int len){
    int addr = p_params[0];
    p_idx = 1;
    while(len > 0){
        int SERVO = id-device_id;
        if(addr == AX_ID)
            device_id = p_params[p_idx]-id+device_id;
        else if(addr == AX_BAUD_RATE){
            if(setBaud(p_params[p_idx]) < 0){
                statusPacket(id,ERR_INSTRUCTION,NULL,0);
                return; 
            }
        }else if(addr == AX_TORQUE_ENABLE){
            servos[SERVO].torque = p_params[p_idx];
        }else if(addr == AX_LED)
            // TODO: update LED
            servos[SERVO].led = p_params[p_idx];
        else if(addr==AX_GOAL_POSITION_L)
            servos[SERVO].goal_l = p_params[p_idx];
        else if(addr==AX_GOAL_POSITION_H){
            servos[SERVO].goal_h = p_params[p_idx];  
            // recalc servo
            //servos[SERVO].output = 952 + ((servos[SERVO].goal_l + (servos[SERVO].goal_h<<8))*4);
            // 180 degrees output, scale +/-307 servo value to +/-90 degrees (+/-500ms,1000ticks)
            servos[SERVO].output = servos[SERVO].center + (((servos[SERVO].goal_l+(servos[SERVO].goal_h<<8)-512) * 10)/3);
            if( servos[SERVO].output < 4000 && servos[SERVO].output > 2000)
                servos[SERVO].torque = 1;    
            else
                servos[SERVO].torque = 0;
        }else
            shared_table[addr] = p_params[p_idx];
        addr++;
        p_idx++;
        len--;
    }
    if(RETURN_LEVEL > RETURN_READ) 
        statusPacket(id,0,0,0);
}

/******************************************************************************
 * Main
 */

int main(){
    // Start up -- read in EEProm variables
    int baud = 1;
    device_id = 12; // TODO: make this EEpromable

    // Setup Serial
    rx_head = rx_tail = 0;
    setBaud(baud);
    setRX();

    // Set default servo params
    servos[0].center = 3000;
    servos[1].center = 3230;
    servos[2].center = 2670;
    servos[3].center = 3000;
    servos[4].center = 3150;
    servos[5].center = 3032;
    servos[6].center = 3000;
    servos[7].center = 3000;

    int i;
    for(i=0; i<8; i++){
        servos[i].led = 0;
        servos[i].goal_l = 0;
        servos[i].goal_h = 2;       // 512 = centered
        servos[i].output = 3000;    // 1500us
    }

    // Setup timer 1 for servos
    TCCR1A = 0;                 // normal mode
    TCCR1B = (1<<CS11);         // prescaler = 8
    TCNT1 = 0;             
    //TIMSK1 |= (1<<OCIE1A);      // enable output compare
    channel = -1;
    // Setup ports
    DDRC = 0xff;
    DDRB = 0x18;
    // enable interrupts
    sei(); 

    int idx = -1;
    while(1){
        // Serial processing
        int x = read();
        if(x != -1){
            if(idx == -1){          // waiting for new packet
                if(x == 0xff)
                    idx++;
            }else if(idx == 0){     // waiting for second 0xFF        
                if(x == 0xff)
                    idx++;
                else
                    idx = -1;
            }else if(idx == 1){     // waiting for ID
                if( x!= 0xff ){        
                    p_id = x;
                    p_chk = x;
                    idx++;
                }
            }else if(idx == 2){     // waiting for length
                p_len = x-2;        // = num of params + 2
                p_chk += x;
                idx++;  
            }else if(idx == 3){     // waiting for instruction
                p_ins = x;
                p_chk += x;
                p_idx = 0;
                idx++;  
            }else if(p_len > 0){    // waiting for params
                p_params[p_idx++] = x;
                p_chk += x;
                p_len--;
            }else{                  // waiting for checksum
                rx_head = rx_tail = 0;
                p_chk += x;
                if((p_id>=device_id) && (p_id<(device_id+8))){
                    if(p_chk%256 == 255){   // process packet
                        if(p_ins == INSTR_PING){
                            statusPacket(p_id,0,NULL,0);
                        }else if(p_ins == INSTR_READ_DATA){
                            doRead(p_id);
                        }else if(p_ins == INSTR_WRITE_DATA){  
                            doWrite(p_id, p_idx-1);
                        }else if(RETURN_LEVEL > RETURN_NONE){
                            statusPacket(p_id,ERR_INSTRUCTION,NULL,0);
                        }
                    }else{                  // report checksum error
                        statusPacket(p_id,ERR_CHECKSUM,NULL,0);
                    }
                }else if((p_id==254) && (p_chk%256 == 255)){ 
                    // TODO: SYNC WRITE!
                }
                idx = -1;
            }
        }   
        // Servo Update
        if(TCNT1 > OCR1A)
            doServo();
    }    
    return 0;
}
