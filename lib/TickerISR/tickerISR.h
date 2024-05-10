#ifndef TICKERISR_H
#define TICKERISR_H

#include <Arduino.h>
#include <Ticker.h>
#include "StateMachine.h"
#include "CAN_PIDs.h"

void setup_ticker(void);
bool checkPID(void);
void PIDs_01sec(void);
void PIDs_05sec(void);
void PIDs_1sec(void);
void PIDs_5sec(void);
void PIDs_10sec(void);
void PIDs_30sec(void);
void PIDs_1min(void);
void PIDs_5min(void);

void PIDs_once(void);

#endif