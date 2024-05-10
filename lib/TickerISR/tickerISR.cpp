#include "tickerISR.h"

bool Print_in_serial = false; // Put True or False to enable SerialPrint of checkPID
Ticker ticker01sec,
       ticker05sec,
       ticker1sec,  
       ticker5sec,
       ticker10sec,
       ticker30sec,
       ticker1min,  // atualizar inserts
       ticker5min,
       tickerONCE;


void setup_ticker()
{
  do 
  {
    Serial.println("Check the PID support...");
  } while(checkPID());
  
  ticker01sec.attach(0.1, PIDs_01sec);
  ticker05sec.attach(0.5, PIDs_05sec);
  ticker1sec.attach(1.0, PIDs_1sec);
  ticker5sec.attach(5.0, PIDs_5sec);
  ticker10sec.attach(10.0, PIDs_10sec);
  ticker30sec.attach(30.0, PIDs_30sec);
  ticker5min.attach(60.0, PIDs_1min);
  ticker5min.attach(300.0, PIDs_5min);
  tickerONCE.once(1.0, PIDs_once);

}

bool checkPID()
{
  // Flag to check if you received the PID support message
  bool check_receive_pid = false;
  unsigned char Data[8] = {0x04, 0x01, 0x00/*=ID*/, 0x00, 0x00, 0x00, 0x00, 0x00};

  for(int i = 1; i < 5; i++)
  {
    Serial.printf("Trying to send PID%d support, please turn on the car electronics\r\n", i);
    check_receive_pid = false;

    while(!check_receive_pid)
    {
      if(i==1)
      {
        Data[2] = PIDsupported1;
        while(!checkReceive())
        {
          if(send_msg(Data) && Print_in_serial) debug_print(Data);
          vTaskDelay(100);          
        }
        MsgRec_CANroutine();
        check_receive_pid = true;
      }

      if(i==2)
      {
        Data[2] = PIDsupported2;
        //if(send_msg(Data) && Print_in_serial) debug_print(Data);
        while(!checkReceive())
        {
          if(send_msg(Data) && Print_in_serial) debug_print(Data);
          vTaskDelay(100);          
        }
        MsgRec_CANroutine();
        check_receive_pid = true;
      }

      if(i==3)
      {
        Data[2] = PIDsupported3;
        //if(send_msg(Data) && Print_in_serial) debug_print(Data);
        while(!checkReceive())
        {
          if(send_msg(Data) && Print_in_serial) debug_print(Data);
          vTaskDelay(100);          
        }
        MsgRec_CANroutine();
        check_receive_pid = true;
      }

      if(i==4)
      {
        Data[2] = PIDsupported4;
        //if(send_msg(Data) && Print_in_serial) debug_print(Data);
        while(!checkReceive())
        {
          if(send_msg(Data) && Print_in_serial) debug_print(Data);
          vTaskDelay(100);          
        }
        MsgRec_CANroutine();
        check_receive_pid = true;
      }

      vTaskDelay(1);
    }
     
    if((i+1)==5) break;
  }

  return false;
}

void PIDs_01sec()
{ 
  insert(TimingAdvance);
  insert(ThrottlePosition);
  insert(RelativeThrottlePosition);
  insert(AbsoluteThrottlePositionB);
  insert(AbsoluteThrottlePositionC); 
  insert(AcceleratorPedalPositionD);
  insert(AcceleratorPedalPositionE);
  insert(AcceleratorPedalPositionF);
  insert(CommandedThrottleActuator);
  insert(RelativeAcceleratorPedalPosition);
  insert(CommandedThrottleActuator2Position);
  

  /*===== Id for bit encode ====== */
    //insert(MonitorStatus);
    //insert(CommandedSecondaryAirStatus);
    //insert(OBDstandard);
    //insert(OxygenSensorsPresent2);                                                
    //insert(AuxiliaryInputStatus);
    //insert(OxygenSensorsPresent);
    //insert(OBDstandard);
    //insert(MonitorStatusDriveCycle);
    //insert(EmissionRequirements);
    //insert(AuxiliaryInput);
    //insert(FuelSystem);

  /*====== Id for test ============*/
   /* ===== PID RESQUEST 1 ===== */                                    
    /*                                         
      insert(EngineLoad);                                        
      insert(EngineCollantTemp);                                         
      insert(ShortTermFuel_Bank1);                                       
      insert(LongTermFuel_Bank1);                                        
      insert(ShortTermFuel_Bank2);                                       
      insert(LongTermFuel_Bank2);                                        
      insert(FuelPressure);                                          
      insert(IntakeManifoldAbsolutePressure);                                      
      insert(EngineRPM);                                         
      insert(VehicleSpeed);                                          
      insert(TimingAdvance);                                         
      insert(IntakeAirTemperature);                                          
      insert(MAFairFlowRate);                                        
      insert(ThrottlePosition);                                          
      insert(CommandedSecondaryAirStatus);                       
      insert(OxygenSensorsPresent);                              
      insert(OxygenSensorVolt_ShortTermFuelTrim_Bank1Sensor1);                           
      insert(OxygenSensorVolt_ShortTermFuelTrim_Bank1Sensor2);                                                  
      insert(OxygenSensorVolt_ShortTermFuelTrim_Bank1Sensor3);                                              
      insert(OxygenSensorVolt_ShortTermFuelTrim_Bank1Sensor4);                                              
      insert(OxygenSensorVolt_ShortTermFuelTrim_Bank2Sensor1);                                              
      insert(OxygenSensorVolt_ShortTermFuelTrim_Bank2Sensor2);                                              
      insert(OxygenSensorVolt_ShortTermFuelTrim_Bank2Sensor3);                                              
      insert(OxygenSensorVolt_ShortTermFuelTrim_Bank2Sensor4);                                              
      insert(OBDstandard);                                              
      insert(OxygenSensorsPresent2);                                                
      insert(AuxiliaryInputStatus);                                                 
      insert(RunTimeSinceEngineStart);                                              
      insert(PIDsupported2);                                                
      insert(DistanceTraveledMIL);                                                 
      insert(FuelRailPressure_vac);                                                 
      insert(FuelRailPressure_dis);                               
    */
  /* ===== PID RESQUEST 2  ===== */
  // B8 = 10111000
    /*
    insert(DistanceTraveledSinceCodeCleared);
    insert(VaporPressure);
    insert(BarometricPressure);
    insert(O2S1_WR_lambda2);
    insert(O2S2_WR_lambda2);
    insert(O2S3_WR_lambda2);
    insert(O2S4_WR_lambda2);
    insert(O2S5_WR_lambda2);
    */ 
  // 01 = 0000000 /1
    //insert(O2S6_WR_lambda2);
    //insert(O2S7_WR_lambda2);
    //insert(O2S8_WR_lambda2);
    //insert(CatalystTemperature_Bank1Sensor1);
    //insert(CatalystTemperature_Bank2Sensor1);
    //insert(CatalystTemperature_Bank1Sensor2);
    //insert(CatalystTemperature_Bank2Sensor2);
  /* ===== PID RESQUEST 3  ===== */
  // 48 = 0/ 1001000
    //insert(ControlModuleVoltage);
    //insert(AbsoluteLoadValue);
    //insert(CommandEquivalenceRatio);
    //insert(RelativeThrottlePosition);
    //insert(AmbientAirTemperature);
    //insert(AbsoluteThrottlePositionB);
    //insert(AbsoluteThrottlePositionC);    
  // D2 = 11010010
    //insert(AcceleratorPedalD);
    //insert(AcceleratorPedalE);
    //insert(AcceleratorPedalF);
    //insert(CommandedThrottleActuator);
    //insert(TimeRun_MIL);
    //insert(TimeSinceTroubleCodesCleared);
    //insert(MaximumValueForEquivalenceRatio);
    //insert(MaximumValueForAirFlowRate);
  // 00 = 00000000
    //insert(FuelType);
    //insert(EthanolFuel);
    //insert(AbsoluteVapourPressure);
    //insert(EvapSystemVaporPressure);
    //insert(ShortTermSecondaryOxygenSensor_bank1bank3);
    //insert(LongTermSecondaryOxygenSensor_bank1bank3);
    //insert(ShortTermSecondaryOxygenSensor_bank2bank4);
    //insert(LongTermSecondaryOxygenSensor_bank2bank4);
  // 01 = 0000000 /1
    //insert(AbsoluteFuelRailPressure);
    //insert(RelativeAcceleratorPedalPosition);
    //insert(HybridBatteryLife);
    //insert(EngineOilTemperature);
    //insert(FuelInjectionTiming);
    //insert(EngineFuelRate);
  /* ===== PID RESQUEST 4  ===== */
  // 00 = 00000000
    //insert(DriverDemandEngine);
    //insert(ActualEngine_PercentTorque);
    //insert(EngineReferenceTorque);
    //insert(EnginePercentTorque);    
    //insert(MassAirFlowSensor);
    //insert(EngineCoolantTemperature);
    //insert(IntakeAirTemperatureSensor);    
  // 00 = 00000000 
    //insert(CommandedEGR_ERROR);
    //insert(CommandedDiesel);
    //insert(ExhaustGasRecircuilationTemperature);
    //insert(CommandedThrottleActuator2Position);
    //insert(FuelPressureControlSystem);
    //insert(InjectionPressureControl);
    //insert(TurboChargerCompressorPressure);
    //insert(BoostPressureControl);
  // 0A = 00001010
    //insert(VGT);
    //insert(WastegateControl);
    //insert(ExhaustPressure);
    //insert(TurbochargerRPM);
    //insert(TurbochargerTemperature1);
    //insert(TurbochargerTemperature2);
    //insert(ChargeAIR_CACT);
    //insert(EGT_Bank1);
  // 10 = 00010000
    //insert(EGT_Bank2);
    //insert(DPF1);
    //insert(DPF2);
    //insert(DPF_Temperature);
    //insert(NOxNTE);
    //insert(PMxNTE);
    //insert(EngineRunTime);
}

void PIDs_05sec()
{
  insert(EngineRPM);
  insert(VehicleSpeed);
  //insert(FuelLevel_PID);
  //insert(EngineCoolant_PID);  

}

void PIDs_1sec()
{
  insert(EngineLoad);
  insert(IntakeManifoldAbsolutePressure);
  insert(MAFairFlowRate);  

  insert(VaporPressure);
  insert(MaximumValueForEquivalenceRatio);
  insert(MaximumValueForAirFlowRate);
  insert(DriverDemandEngine);
  insert(ActualEngine_PercentTorque);
  insert(EngineReferenceTorque);
  insert(EnginePercentTorque);

  insert(MassAirFlowSensor);
  insert(CommandedDiesel);
  insert(TurbochargerRPM);
  insert(RunTimeSinceEngineStart);
  
}


void PIDs_5sec()
{
  insert(O2S1_WR_lambda2);
  insert(O2S2_WR_lambda2);
  insert(O2S3_WR_lambda2);
  insert(O2S4_WR_lambda2);
  insert(O2S5_WR_lambda2);
  insert(O2S6_WR_lambda2);
  insert(O2S7_WR_lambda2);
  insert(O2S8_WR_lambda2);

  insert(OxygenSensorsPresent);                              
  insert(OxygenSensorVolt_ShortTermFuelTrim_Bank1Sensor1);                           
  insert(OxygenSensorVolt_ShortTermFuelTrim_Bank1Sensor2);                                                  
  insert(OxygenSensorVolt_ShortTermFuelTrim_Bank1Sensor3);                                              
  insert(OxygenSensorVolt_ShortTermFuelTrim_Bank1Sensor4);                                              
  insert(OxygenSensorVolt_ShortTermFuelTrim_Bank2Sensor1);                                              
  insert(OxygenSensorVolt_ShortTermFuelTrim_Bank2Sensor2);                                              
  insert(OxygenSensorVolt_ShortTermFuelTrim_Bank2Sensor3);                                              
  insert(OxygenSensorVolt_ShortTermFuelTrim_Bank2Sensor4);    

  insert(CommandEquivalenceRatio);

  insert(ShortTermSecondaryOxygenSensor_bank1bank3);
  insert(LongTermSecondaryOxygenSensor_bank1bank3);
  insert(ShortTermSecondaryOxygenSensor_bank2bank4);
  insert(LongTermSecondaryOxygenSensor_bank2bank4);

  insert(FuelRailPressure_vac);                                                 
  insert(FuelRailPressure_dis); 
  
  
}

void PIDs_10sec(){
  insert(EngineCollantTemp);
  insert(IntakeAirTemperature);

  insert(CatalystTemperature_Bank1Sensor1);
  insert(CatalystTemperature_Bank2Sensor1);
  insert(CatalystTemperature_Bank1Sensor2);
  insert(CatalystTemperature_Bank2Sensor2);

  
  insert(BarometricPressure);
  insert(AmbientAirTemperature);

  insert(AbsoluteVapourPressure);
  insert(EvapSystemVaporPressure);
  insert(EngineOilTemperature);

  insert(EngineCoolantTemperature);
  insert(IntakeAirTemperatureSensor); 
  insert(ExhaustGasRecircuilationTemperature);

  insert(TurboChargerCompressorPressure);
  insert(BoostPressureControl);
  insert(VGT);
  insert(WastegateControl);
  insert(ExhaustPressure);

  insert(TurbochargerTemperature1);
  insert(TurbochargerTemperature2);
  insert(ChargeAIR_CACT);
  insert(EGT_Bank1);
  insert(EGT_Bank2);
  insert(DPF_Temperature);
  insert(NOxNTE);
  insert(PMxNTE);
}

void PIDs_30sec()
{
  insert(ControlModuleVoltage);
  insert(AbsoluteLoadValue);
  insert(AbsoluteFuelRailPressure);
  insert(FuelPressureControlSystem);
  insert(InjectionPressureControl);
  insert(DPF1);
  insert(DPF2);
  
}

void PIDs_1min(void){  

  insert(FuelPressure);  

  insert(TimeRun_MIL);
  insert(TimeSinceTroubleCodesCleared);
  insert(CommandedEGR_ERROR);
  insert(EngineRunTime);

  insert(ShortTermFuel_Bank1);                                       
  insert(LongTermFuel_Bank1);                                        
  insert(ShortTermFuel_Bank2);                                       
  insert(LongTermFuel_Bank2);
  

}

void PIDs_5min(void){
  insert(DistanceTraveledSinceCodeCleared);
  insert(DistanceTraveledMIL); 
  insert(EthanolFuel);

}
void PIDs_once(void){
  insert(FuelType);
  insert(HybridBatteryLife);
}
