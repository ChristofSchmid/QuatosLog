/*
    This file is part of AutoQuad ESC32.

    AutoQuad ESC32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad ESC32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad ESC32.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#include "esc32.h"


#include <stdio.h>
//#include <unistd.h>
#include <string.h>

#include <fcntl.h>
#include <getopt.h>
#include <stdlib.h>
//#include <pthread.h>
#include <math.h>
#include <signal.h>

#define ESC32_DEBUG

//#define EIGEN_NO_DEBUG
//#define EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_PARALLELIZE  // don't use openmp




#define DEFAULT_MAX_AMPS        30.0
#define MAX_TELEM_STORAGE	200000

unsigned char checkInA, checkInB;
unsigned char checkOutA, checkOutB;
char sendBuf[128];
unsigned char sendBufPtr;
volatile float telemValueAvgs[BINARY_VALUE_NUM];
volatile float telemValueMaxs[BINARY_VALUE_NUM];
volatile float telemData[256][BINARY_VALUE_NUM];
float *telemStorage;
volatile int telemStorageNum;
float maxAmps;

volatile unsigned char lastAck;
volatile unsigned short lastSeqId = -1;
volatile short paramId;

unsigned short commandSeqId = 1;

char port[256];
unsigned int baud;

void esc32Send(void) {
	serialWrite( sendBuf, sendBufPtr);
}

void esc32OutChecksum(unsigned char c) {
	checkOutA += c;
	checkOutB += checkOutA;
}

void esc32InChecksum(unsigned char c) {
	checkInA += c;
	checkInB += checkInA;
}

unsigned char esc32GetChar() {
	unsigned char c;

	c = serialRead();
	esc32InChecksum(c);

	return c;
}

void esc32SendChar(unsigned char c) {
	sendBuf[sendBufPtr++] = c;
	esc32OutChecksum(c);
}

void esc32SendShort(unsigned short i) {
	unsigned char j;
	unsigned char *c = (unsigned char *)&i;

	for (j = 0; j < sizeof(short); j++)
		esc32SendChar(*c++);
}

void esc32SendFloat(float f) {
	unsigned char j;
	unsigned char *c = (unsigned char *)&f;

	for (j = 0; j < sizeof(float); j++)
		esc32SendChar(*c++);
}

unsigned short esc32GetShort() {
	unsigned short d;
	unsigned char *c = (unsigned char *)&d;
	unsigned int i;

	for (i = 0; i < sizeof(unsigned short); i++)
		*c++ = esc32GetChar();

	return d;
}

float esc32GetFloat() {
	float f;
	unsigned char *c = (unsigned char *)&f;
	unsigned int i;

	for (i = 0; i < sizeof(float); i++)
		*c++ = esc32GetChar();

	return f;
}

void *esc32Read(void *ipt) {
	unsigned short seqId;
        unsigned char c;
	int rows, cols;
	int n;
        int i, j;

        while (1) {
		thread_read_start:

		c = esc32GetChar();
		if (c != 'A')
			goto thread_read_start;

		c = esc32GetChar();
		if (c != 'q')
			goto thread_read_start;

		c = esc32GetChar();
		checkInA = checkInB = 0;

		if (c == 'C') {
			n = esc32GetChar();	// count
			c = esc32GetChar();	// command
			seqId = esc32GetShort();

			if (n > 3)
				paramId = esc32GetShort();

			if (serialRead() != checkInA)
				goto thread_read_start;

			if (serialRead() != checkInB)
				goto thread_read_start;

			if (c == BINARY_COMMAND_ACK) {
				lastSeqId = seqId;
				lastAck = 1;
#ifdef ESC32_DEBUG
Serial.print("Ack ");
Serial.println(seqId);

//				printf("Ack [%d]\n", seqId);
#endif
			}
			else if (c == BINARY_COMMAND_NACK) {
				lastSeqId = seqId;
				lastAck = 0;
#ifdef ESC32_DEBUG
Serial.print("Nack ");
Serial.println(seqId);
//				printf("Nack [%d]\n", seqId);
#endif
			}
			else if (c == BINARY_COMMAND_GET_PARAM_ID) {
				lastSeqId = seqId;
			}
			else {
				printf("Unkown command [%d]\n", c);
			}
		}
		if (c == 'T') {
			rows = esc32GetChar();
			cols = esc32GetChar();

			for (i = 0; i < rows; i++)
				for (j = 0; j < cols; j++)
					telemData[i][j] = esc32GetFloat();

			if (serialRead() != checkInA)
				goto thread_read_start;

			if (serialRead() != checkInB)
				goto thread_read_start;

			// update averages
			for (i = 0; i < rows; i++)
				for (j = 0; j < cols; j++)
					telemValueAvgs[j] -= (telemValueAvgs[j] - telemData[i][j]) * 0.01;

			// update max values
			for (i = 0; i < rows; i++)
				for (j = 0; j < cols; j++)
					if (telemValueMaxs[j] < telemData[i][j])
						telemValueMaxs[j] = telemData[i][j];

			// save to memory
			for (i = 0; i < rows; i++) {
				for (j = 0; j < cols; j++)
					telemStorage[MAX_TELEM_STORAGE*j + telemStorageNum] = telemData[i][j];
				telemStorageNum++;
			}

			// output to stream
			
		}
	}
}

unsigned short esc32SendCommand(unsigned char command, float param1, float param2, int n) {
	checkOutA = checkOutB = 0;
	sendBufPtr = 0;

#ifdef ESC32_DEBUG
Serial.print("Send ");
Serial.print(command);
Serial.print(" ");
Serial.println(commandSeqId);
//        printf("Send %d [%d] - ", command, commandSeqId);
#endif
        sendBuf[sendBufPtr++] = 'A';
        sendBuf[sendBufPtr++] = 'q';
        esc32SendChar(1 + 2 + n*sizeof(float));
        esc32SendChar(command);
        esc32SendShort(commandSeqId++);
        if (n > 0)
                esc32SendFloat(param1);
        if (n > 1)
                esc32SendFloat(param2);
        sendBuf[sendBufPtr++] = checkOutA;
        sendBuf[sendBufPtr++] = checkOutB;
        esc32Send();
#ifdef ESC32_DEBUG
Serial.print("Bytes ");
Serial.println(sendBufPtr);

//        printf("%d bytes\n", sendBufPtr);

#endif

	return (commandSeqId - 1);
}



int esc32SendReliably(unsigned char command, float param1, float param2, int n) {
	unsigned short seqId;
	int ret = 0;
	int i, j;

	j = 0;
	do {
		seqId = esc32SendCommand(command, param1, param2, n);

		i = 0;
		do {
			delay(10);
			i++;
		} while (lastSeqId != seqId && i < 500);

		j++;
	} while (lastSeqId != seqId && j < 5);

	if (lastSeqId == seqId)
		ret = lastAck;

	return ret;
}

int16_t esc32GetParamId(const char *name) {
	unsigned short seqId;
	int id;
	int i, j, k;

	j = 0;
	do {
		seqId = commandSeqId++;

		checkOutA = checkOutB = 0;
		sendBufPtr = 0;

		sendBuf[sendBufPtr++] = 'A';
		sendBuf[sendBufPtr++] = 'q';
		esc32SendChar(1 + 16 + 2);
		esc32SendChar(BINARY_COMMAND_GET_PARAM_ID);
		esc32SendShort(seqId);
		for (i = 0; i < 16; i++)
			esc32SendChar(name[i]);

		sendBuf[sendBufPtr++] = checkOutA;
		sendBuf[sendBufPtr++] = checkOutB;
		esc32Send();

		k = 0;
		do {
			delay(10);
			k++;
		} while (lastSeqId != seqId && k < 500);

		j++;
	} while (lastSeqId != seqId && j < 5);

	if (lastSeqId == seqId)
		id = paramId;
	else
		id = -1;

	return id;
}

short esc32SetParamByName(const char *name, float value) {
	short int paramId;

	paramId = esc32GetParamId(name);

	if (paramId < 0)
		return -1;

	
	return esc32SendReliably(BINARY_COMMAND_SET, paramId, value, 2);
}



void signal_callback_handler(int signum) {
	printf("Caught signal %d\n",signum);
	esc32SendReliably(BINARY_COMMAND_DISARM, 0.0, 0.0, 0);
	esc32SendCommand(BINARY_COMMAND_CLI, 0.0, 0.0, 0);
	exit(signum);
}

int Test(){
	esc32SendReliably(BINARY_COMMAND_ARM, 0.0, 0.0, 0);
	esc32SendReliably(BINARY_COMMAND_STOP, 0.0, 0.0, 0);
	esc32SendReliably(BINARY_COMMAND_TELEM_RATE, 0.0, 0.0, 1);
	esc32SendReliably(BINARY_COMMAND_TELEM_VALUE, 0.0, BINARY_VALUE_RPM, 2);
	esc32SendReliably(BINARY_COMMAND_TELEM_VALUE, 1.0, BINARY_VALUE_VOLTS_MOTOR, 2);
	esc32SendReliably(BINARY_COMMAND_TELEM_VALUE, 2.0, BINARY_VALUE_AMPS, 2);
//	esc32SendReliably(BINARY_COMMAND_SET, MAX_CURRENT, 0.0, 2);
	esc32SetParamByName("MAX_CURRENT", 0.0);


	// disarm motor if interrupted
	signal(SIGINT, signal_callback_handler);

	esc32SendReliably(BINARY_COMMAND_TELEM_RATE, 0.0, 0.0, 1);
	//esc32SendCommand(BINARY_COMMAND_CLI, 0.0, 0.0, 0);

	return 1;
}

int Test1(){
     char cBuff[256];
     int i = 0;

     //esc32SendCommand(BINARY_COMMAND_CLI, 0.0, 0.0, 0);
     
     esc32SendReliably(BINARY_COMMAND_TELEM_RATE, 10.0, 0.0, 1);

     esc32SendReliably(BINARY_COMMAND_TELEM_VALUE, 0.0, BINARY_VALUE_RPM, 2);
     while ( Serial1.available() ){
       
       cBuff[i] = serialRead();
       i++;
  
    }
Serial.print("RPM: ");
Serial.println(cBuff);
  
  }
  
void serialWriteChar(const char* c) {
	char ret;

	ret = Serial1.write( c, 1);
}

void serialWrite(char *str, unsigned int len) {
	char ret;

	ret = Serial1.write(str, len);
}

void serialPrint( char *str) {
	Serial1.write( str, strlen(str));
}

unsigned char serialAvailable() {
	return Serial1.available();
}

void serialFlush() {
	Serial1.flush();
}

unsigned char serialRead() {
	char ret;
	unsigned char c;
	c = Serial1.read();
	return c;
}
