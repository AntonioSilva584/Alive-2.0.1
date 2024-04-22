#include <Arduino.h>
#include "wdt.h"
#include "BLE.h"
#include "can_defs.h"
#include "StateMachine.h"
#include "message.h"
#include "tickerISR.h"
#include "CAN_PIDs.h"

boolean flagCANInit = false;   // If false indicates that the CAN module was not initialized successfully
boolean led_flag = false;
state_t state = IDLE_ST;
uint32_t initialTime = 0;
TaskHandle_t CANtask = NULL, BLEtask = NULL;

/* State Machine Functions */
void logCAN(void* arg);
void BLElog(void* arg);

void setup()
{    
  Serial.begin(9600);
  Serial.println("INICIANDO ALIVE.");
  
  pinMode(LED_BUILTIN, OUTPUT);

  // if there was an error in the CAN it shows
  flagCANInit = can_setup();
  if(!flagCANInit)
  {
    Serial.println("CAN error!!!");
    esp_restart();
  }

  set_mask_filt();
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);

  setup_BLE();

  setup_ticker();

  xTaskCreatePinnedToCore(logCAN, "CANstatemachine", 4096, NULL, 5, &CANtask, 0);
  xTaskCreatePinnedToCore(BLElog, "BLEstatemachine", 4096, NULL, 4, &BLEtask, 1);

  setupWDT();
}

void loop() { reset_rtc_wdt(); /* Reset the wathdog timer */ }

void logCAN(void* arg)
{
  while(1)
  {
    if(flagCANInit)
    {
      state = CircularBuffer_state();

      initialTime = millis();    
      while(!checkReceive() && state!=IDLE_ST)
      {
        if(millis() - initialTime >= 4000) break;
        vTaskDelay(1);
      }
    }

    if(checkReceive() && flagCANInit)
    {
      // Routine that handles when a message arrives via can
      MsgRec_CANroutine(); 
    }
  }

  vTaskDelay(1);
}

void BLElog(void* arg)
{
  BLEmsg_t ble = defaultmsg();

  for(;;)
  { 
    while(BLE_connected())
    {
      ble = requestMsg();
      BLE_Sender(&ble, sizeof(ble));

      led_flag = !led_flag;
      digitalWrite(LED_BUILTIN, led_flag);
      vTaskDelay(MAX_BLE_DELAY);
    }

    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(1);
  }
}
