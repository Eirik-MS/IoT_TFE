// Incluye las librerías necesarias para el módem y la comunicación MQTT
// Requiere instalar Librerias: StreamDebugger y TimerOne desde libray Manager
#include <RIC3D.h>
#include <RIC3DMODEM.h>
#include <TimerOne.h>
#include <Arduino.h>

#include "modem.h"
#include "utility.h"
#include "pwm_speed.h"

// Definiciones de pines y variables globales
#define EMA_FILTER_WEIGTH   0.5 //un numero entre 0 a 1
#define ORDER_FILTER 5
#define SerialMon Serial     // Monitor serial para depuración
#define SerialAT Serial3     // Comunicación con el módem 4G

#define MAXIMUM_TEMPERATURE 29.0

#define BUS_MAX_CAPACITY 50

RIC3DMODEM gModem;

// Configuración del módem (ajustar según el hardware y proveedor)
const char apn[]      = "grupotesacom.claro.com.ar";
const char apnUser[] = "";
const char apnPassword[] = "";

const char mqttHost[]   = "10.25.1.152";          // host de tdata
const int mqttPort = 4098;                        // puerto
const char mqttUser[]   = "8YVlOfVANIaKQ961f6k8"; // aca se debe poner el token del device de tdata // aa1f3770-8dcd-11f0-810b-393772542f99
const char *mqttPassword = NULL; //8YVlOfVANIaKQ961f6k8

// Module baud rate
uint32_t rate = 115200;

// Select SIM Card (0 = right, 1 = left)
bool sim_selected = 1;

// Variables para sensores

uint8_t doors_state = LOW;

float temperature = 0.0;
// Añadir más variables según los sensores utilizados

// Variables para filtros y estadísticas
const int orden_filtro = ORDER_FILTER;  // Puede ser 1 para filtro de primer orden o 5 para media móvil
float buffer_sensor_temperature[orden_filtro] = {0};

// Añadir más buffers si es necesario

float filtered_value_temperature = 0.0;

float sum_temperature = 0.0;

int measurement_count = 0;

float average_temperature = 0.0;

unsigned long passengerPulses = 0;

float confort = 0.0;
float occupation_porcentage = 0.0;

// Variables para gestión de alarmas y errores
bool thermistor_disconnected_loop = false;
bool max_temperature_level_reached = false;

bool door_open = false;

bool puerta_abierta_en_movimiento = false;

bool sobrecarga = false;
// Añadir más flags según sea necesario

// Temporizadores
unsigned long tiempo_medicion = 0;
const unsigned long intervalo_medicion = 1000;  // Intervalo para mediciones (1 segundo)

unsigned long tiempo_reporte = 0;
const unsigned long intervalo_reporte = 10000; // Intervalo para reportes (10 segundos)

//-------------------------------------
volatile long lastLedBlink = 0;

volatile uint8_t lastDI1 = LOW;
volatile uint8_t lastDI2 = LOW;
volatile uint8_t lastDI3 = LOW;
volatile uint32_t passengerUpCounter = 0;

volatile uint32_t passengerDownCounter = 0;

//-------------------------------------
WaveStats pwm_data = {0.0f, 0.0f, false};

Motion speed_accs_data = {0.0f, 0.0f};


//-------------------------------------
void modemSetup(){
  SerialMon.println(F("modemSetup()"));
  pinMode(SIM_SELECT, OUTPUT);
  SerialAT.begin(rate);
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

// Funciones para aplicar filtros
float aplicarFiltroOrden1(float valor_actual, float valor_anterior) {
    SerialMon.println(F("Usar filtro de Primer Orden"));
    return valor_actual * EMA_FILTER_WEIGTH + valor_anterior * (1 - EMA_FILTER_WEIGTH);
}

float aplicarMediaMovil(float* buffer, float nuevo_valor) {
    SerialMon.println(F("Filtro Media Movil"));
    float sum = 0; 

    //shift values in the buffer first
    for (int j = orden_filtro - 1; j > 0; j--){
      buffer[j] = buffer[j-1];
    }
    buffer[0] = nuevo_valor;

    int cant = measurement_count < orden_filtro ? measurement_count : orden_filtro;
    //calculate the sum for the filtered value 
    for (int i = 0; i < cant; i++){
      sum += buffer[i]; 
    }

    return sum / cant;
}


//-------------------------------------
void timer1ISR(void) {

uint8_t di1 = digitalRead(DI1);
  if (di1 != lastDI1) {                     // state changed?
    if (lastDI1 == HIGH && di1 == LOW) {    // falling edge
      passengerUpCounter++;                 // one more passenger up
    }
    lastDI1 = di1;
  }

  uint8_t di3 = digitalRead(DI3);
  if (di3 != lastDI3) {                     // state changed?
    if (lastDI3 == HIGH && di3 == LOW) {    // falling edge
      passengerDownCounter++;               // one more passenger down
    }
    lastDI3 = di3;
  }
    //led blink
  if (millis() - lastLedBlink > 1000) {
    lastLedBlink = millis();
    digitalWrite(LED1, !digitalRead(LED1));
    SerialMon.println(F("Blink LED1"));
  }
}


// Funciones para lectura de sensores
void leerSensores() {
  SerialMon.println(F("Lectura de Sensores"));
    // Leer los valores de los sensores conectados
    // Por ejemplo:
    // sensor1_valor = analogRead(PIN_SENSOR1);
    // Realizar conversión de unidades si es necesario

  temperature = analogRead(AI0) / 30.0;
  doors_state = digitalRead(DI2);

  SerialMon.print(F("Midiendo estado de las puertas: "));
  SerialMon.println(doors_state);

  SerialMon.print(F("Midiendo temperatura: "));
  SerialMon.println(temperature);

  passengerPulses += passengerUpCounter;
  passengerUpCounter = 0;

  SerialMon.print(F("Midiendo pulsos de subida de pasajeros: "));
  SerialMon.println(passengerPulses);

  passengerPulses = passengerPulses < passengerDownCounter ? 0 : passengerPulses - passengerDownCounter;
  passengerDownCounter = 0;

  SerialMon.print(F("Midiendo pulsos de bajada de pasajeros: "));
  SerialMon.println(passengerPulses);


  measurement_count++;
}


// Funciones para actualizar estadísticas
void actualizarEstadisticas() {
  SerialMon.println(F("Actualizar Estadistica"));
    // Actualizar máximos, mínimos y suma para promedios
    // Incrementar contador de mediciones

  sum_temperature += filtered_value_temperature;

  //TODO calculate speed based on pulses
  //speed = speed_pulses ...

  average_temperature = sum_temperature / measurement_count;

  confort = max(1, min(10, (10 - 0.4 * abs(filtered_value_temperature - 24) - speed_accs_data.accs)));

  occupation_porcentage = (passengerPulses / BUS_MAX_CAPACITY) * 100;
}

void verificarSensoresDesconetados() {
  SerialMon.println(F("Verificar sensores conectados"));

  if (temperature < 4.0) {
    thermistor_disconnected_loop = true;
  } 
}

// Funciones para detección de alarmas y errores
void verificarAlarmas() {
  SerialMon.println(F("Verificar Alarmas"));
    // Verificar si los valores de los sensores superan umbrales definidos
    // Actualizar los flags de alarma correspondientemente

  if(((filtered_value_temperature - 4.0) * 100.0 / 16.0) - 20 >= MAXIMUM_TEMPERATURE) {
    max_temperature_level_reached = true;
  }

  if (doors_state == LOW) {
    door_open = true;
  }

  if (door_open && speed_accs_data.speed > 5) {
    puerta_abierta_en_movimiento = true;
  }

  if (passengerPulses > BUS_MAX_CAPACITY * 0.9) {
    sobrecarga = true;
  }
}

void enviarAlarmas() {
  SerialMon.println(F("Enviar Alarmas"));
    // Enviar alarmas al broker MQTT si se han detectado
    // Utilizar gmodem.PublishData() o la función correspondiente

  char payload[32];

  if (thermistor_disconnected_loop) {
    gModem.publishData("termistor_desconectado", "true");
  }

  if (max_temperature_level_reached) {
    gModem.publishData("temperatura_maxima", "true");
  }
  
  if (door_open) {
    gModem.publishData("puerta_abierta", "true");
  }

  if (puerta_abierta_en_movimiento) {
    gModem.publishData("puerta_abierta_en_movimiento", "true");
  }

  if (sobrecarga) {
    gModem.publishData("sobrecarga", "true");
  }


  resetearAlarmas();
}

// Funciones para reportes
void enviarReporte() {
  SerialMon.println(F("Enviar Reportes"));
    // Calcular promedios
    // Enviar máximos, mínimos y promedios al broker MQTT

    //cudal pulses
  
  //Have this long for large amounts of data, 
  char value_str_buffer[32];

  // this can be used for safer stuff: snprintf(value_str_buffer, "%4.2f")

  char temperature_str[] = "Temperatura";
  dtostrf(filtered_value_temperature, 4, 2, value_str_buffer);
  gModem.publishData(temperature_str, value_str_buffer);
  SerialMon.print("Temperature sent as data:");
  SerialMon.println(average_temperature);

  char speed_str[] = "Velocidad";
  dtostrf(speed_accs_data.speed, 4, 2, value_str_buffer);
  gModem.publishData(speed_str, value_str_buffer);

  char passenger_str[] = "Cant pasajeros";
  snprintf(value_str_buffer, sizeof(value_str_buffer), "%ld", passengerPulses);
  gModem.publishData(passenger_str, value_str_buffer);

  char confort_str[] = "Confort";
  dtostrf(confort, 4, 2, value_str_buffer);
  gModem.publishData(confort_str, value_str_buffer);

  char accs_str[] = "Acceleracion";
  dtostrf(speed_accs_data.accs, 4, 2, value_str_buffer);
  gModem.publishData(accs_str, value_str_buffer);



}

void resetearEstadisticas() {
  SerialMon.println(F("Resetear Estadisticas"));

  for (int i = 0; i < orden_filtro; i++) {
    buffer_sensor_temperature[i] = 0;
  }

  sum_temperature = 0.0;
  average_temperature = 0.0;

  confort = 0.0;

  measurement_count = 0;
}

void resetearAlarmas() {
  SerialMon.println(F("Resetear Alarmas"));

  thermistor_disconnected_loop = false;
  max_temperature_level_reached = false;

  door_open = false;

  puerta_abierta_en_movimiento = false;
  sobrecarga = false;
}



// Función de setup
void setup() {

  delay(100);
  pinMode(DI0, INPUT_PULLUP); //PWM - speed + accs
  pinMode(DI1, INPUT_PULLUP); //Passanger up
  pinMode(DI2, INPUT_PULLUP); //Door State   
  pinMode(DI3, INPUT_PULLUP); //Passanger Down

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);



  SerialMon.println(F("Inicializar comunicación serial para depuración"));
    // Inicializar comunicación serial para depuración
    SerialMon.begin(115200);

    // Configurar pines de entrada para sensores
    // pinMode(PIN_SENSOR1, INPUT);
    // Añadir configuración de pines según sea necesario

  //módem Setup and init
  modemSetup();
  int result;
  while (result = modemInit()){
    SerialMon.print(F("falló! -> "));
    SerialMon.println(result);
    delay(10000);
  }  
  
  SerialMon.println(F("Inicializacion de Modem"));
    // Inicializar variables y temporizadores
    tiempo_medicion = millis();
    tiempo_reporte = millis();

  analogReference(INTERNAL2V56);
  noInterrupts();
  Timer1.initialize(5000);            //5ms -> 100hz
  Timer1.attachInterrupt(timer1ISR);  // blinkLED to run every 0.15 seconds
  interrupts();
    // Inicializar variables estadísticas y de alarmas
  resetearEstadisticas();
  resetearAlarmas();
}

// Función principal loop
void loop() {
    // Mantener la conexión MQTT
    // Reconectar si es necesario

    // Tomar lecturas periódicas
    if (millis() - tiempo_medicion >= intervalo_medicion) {
        tiempo_medicion = millis();

        // Leer sensores
        leerSensores();

        pwm_data = measureWave(DI0);

        speed_accs_data = waveToMotion(pwm_data);
        SerialMon.print("speed: ");
        SerialMon.println(speed_accs_data.speed);
        SerialMon.print("Accs: ");
        SerialMon.println(speed_accs_data.accs);

        verificarSensoresDesconetados();

        if (!thermistor_disconnected_loop) {
          // Aplicar filtros
          if (orden_filtro == 1) {
            filtered_value_temperature = aplicarFiltroOrden1(temperature, buffer_sensor_temperature[0]);
            buffer_sensor_temperature[0] = filtered_value_temperature;
          } 
          
          if (orden_filtro == 5) {
            filtered_value_temperature = aplicarMediaMovil(buffer_sensor_temperature, temperature);
          }

          SerialMon.print(F("Valor filtrado del nivel del pozo: "));
          SerialMon.println(filtered_value_temperature);

          // Actualizar estadísticas
          // Las estadisticas pueden verse impactadas por el valor filtrado... He adjuntado un pequeño documento al respecto
          actualizarEstadisticas();
        }

        // Verificar alarmas
        verificarAlarmas();

        // Enviar alarmas si corresponde
        enviarAlarmas();

    }

    // Enviar reportes periódicos
    if (millis() - tiempo_reporte >= intervalo_reporte) {
        tiempo_reporte = millis();

        // Enviar reporte al broker MQTT
        enviarReporte();

        // Resetear estadísticas para el próximo período
        resetearEstadisticas();
    }

    // Otras tareas si es necesario
    // Manejo de eventos, comunicación, etc.
}
