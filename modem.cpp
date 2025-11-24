#include <RIC3D.h>
#include <RIC3DMODEM.h>
#include <TimerOne.h>
#include <Arduino.h>

#include "setting.h"

// Select SIM Card (0 = right, 1 = left)
bool sim_selected = 1;

// Configuración del módem (ajustar según el hardware y proveedor)
const char apn[]      = "grupotesacom.claro.com.ar";
const char apnUser[] = "";
const char apnPassword[] = "";

const char mqttHost[]   = "10.25.1.152";          // host de tdata
const int mqttPort = 4098;                        // puerto
const char mqttUser[]   = "8YVlOfVANIaKQ961f6k8"; // aca se debe poner el token del device de tdata // aa1f3770-8dcd-11f0-810b-393772542f99
const char *mqttPassword = NULL; //8YVlOfVANIaKQ961f6k8

//-------------------------------------
void modemSetup(){
  SerialMon.println(F("modemSetup()"));
  pinMode(SIM_SELECT, OUTPUT);
  SerialAT.begin(SerialBAUD);
  //gModem.begin(&SerialAT, &SerialMon);
  gModem.begin(&SerialAT, &SerialMon, true, true);  //depuración + at dump
  digitalWrite(SIM_SELECT, sim_selected);
  
  gModem.turnOff();
  gModem.turnOn();
}

int modemInit(){
  int result;
  SerialMon.println(F("modemInit()"));  
  gModem.test();
  if (result = gModem.test()) return result;
  SerialMon.println(F("ok"));
  gModem.deactivatePDPContext();
  if (result = gModem.createPDPContext(apn, apnUser, apnPassword)) return result;
  SerialMon.println(F("ok"));
  if (result = gModem.activatePDPContext()) return result;
  SerialMon.println(F("ok"));
  if (result = gModem.connectMQTTClient(mqttHost, mqttPort, mqttUser, mqttPassword)) return result;
  SerialMon.println(F("ok"));
  return result;
}

