// Incluye las librerías necesarias para el módem y la comunicación MQTT
// Requiere instalar Librerias: StreamDebugger y TimerOne desde libray Manager
#include <RIC3D.h>
#include <RIC3DMODEM.h>
#include <TimerOne.h>

// Definiciones de pines y variables globales
#define EMA_FILTER_WEIGTH   0.5 //un numero entre 0 a 1
#define ORDER_FILTER 5
#define SerialMon Serial     // Monitor serial para depuración
#define SerialAT Serial3     // Comunicación con el módem 4G

#define MINIMUM_WELL_LEVEL 10.0

RIC3DMODEM gModem;



// Configuración del módem (ajustar según el hardware y proveedor)
const char apn[]      = "grupotesacom.claro.com.ar";
const char apnUser[] = "";
const char apnPassword[] = "";

const char mqttHost[]   = "10.25.1.152";          // host de tdata
const int mqttPort = 4098;                        // puerto
const char mqttUser[]   = "1ywx579ruc54pp3ch33v"; // aca se debe poner el token del device de tdata // aa1f3770-8dcd-11f0-810b-393772542f99
const char *mqttPassword = NULL; //1ywx579ruc54pp3ch33v

// Module baud rate
uint32_t rate = 115200;

// Select SIM Card (0 = right, 1 = left)
bool sim_selected = 1;

// Variables para sensores
/*float sensor1_valor = 0.0;
float sensor2_valor = 0.0;*/
uint8_t pump_state = LOW;

float caudal = 0.0;
float level = 0.0;
// Añadir más variables según los sensores utilizados

// Variables para filtros y estadísticas
const int orden_filtro = ORDER_FILTER;  // Puede ser 1 para filtro de primer orden o 5 para media móvil
float buffer_sensor_caudal[orden_filtro] = {0};  // Buffer para media móvil
float buffer_sensor_level[orden_filtro] = {0};

// Añadir más buffers si es necesario

float filtered_value_caudal = 0.0;
float filtered_value_level = 0.0;

 /*
float max_sensor1 = -INFINITY;
float min_sensor1 = INFINITY;
float sum_sensor1 = 0.0;

float max_sensor2 = -INFINITY;
float min_sensor2 = INFINITY;
float sum_sensor2 = 0.0;
*/

float max_caudal = -INFINITY;
float min_caudal = INFINITY;
float sum_caudal = 0.0;

float max_level = -INFINITY;
float min_level = INFINITY;
float sum_level = 0.0;

int contador_mediciones = 0;

unsigned long caudalPulses = 0;

// Variables para gestión de alarmas y errores
bool flow_meter_disconnected_loop = false;
bool level_meter_disconnected_loop = false;
bool pump_off = false;
bool minimun_level_reached = false;
// Añadir más flags según sea necesario

// Temporizadores
unsigned long tiempo_medicion = 0;
const unsigned long intervalo_medicion = 1000;  // Intervalo para mediciones (1 segundo)

unsigned long tiempo_reporte = 0;
const unsigned long intervalo_reporte = 900000; // Intervalo para reportes (15 minutos)

//-------------------------------------
volatile uint8_t lastDI3 = LOW;
volatile uint32_t pulseCounter = 0;
volatile long lastLedBlink = 0;

volatile uint8_t lastDI0 = LOW;
volatile uint32_t caudalPulseCounter = 0;

//-------------------------------------
void timer1ISR(void) {
  //conteo de pulsos
  uint8_t di3 = digitalRead(DI3);
  if (di3 != lastDI3) {
    lastDI3 = di3;
    pulseCounter++;
    digitalWrite(LED3, !digitalRead(LED3));
  }

  //led blink
  if (millis() - lastLedBlink > 1000) {
    lastLedBlink = millis();
    digitalWrite(LED1, !digitalRead(LED1));
    SerialMon.println(F("Blink LED1"));
  }

  uint8_t di0 = digitalRead(DI0);
  if (di0 != lastDI0) {
    lastDI0 = di0;
    if (di0 == HIGH) {
      caudalPulseCounter++;
    }
  }
}

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


// Funciones para lectura de sensores
void leerSensores() {
  SerialMon.println(F("Lectura de Sensores"));
    // Leer los valores de los sensores conectados
    // Por ejemplo:
    // sensor1_valor = analogRead(PIN_SENSOR1);
    // Realizar conversión de unidades si es necesario

  caudal = analogRead(AI0) / 40.0;
  level = analogRead(AI1) / 40.0;
  pump_state = digitalRead(DI2);

  SerialMon.print(F("Midiendo estado de la bomba: "));
  SerialMon.println(pump_state);

  SerialMon.print(F("Midiendo caudal: "));
  SerialMon.println(caudal);

  SerialMon.print(F("Midiendo nivel: "));
  SerialMon.println(level);
    
  //noInterrupts();
  caudalPulses += caudalPulseCounter;
  caudalPulseCounter = 0;
  //interrupts();

  SerialMon.print(F("Midiendo pulsos: "));
  SerialMon.println(caudalPulses);

  contador_mediciones++;
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

    int cant = contador_mediciones < orden_filtro ? contador_mediciones : orden_filtro;
    //calculate the sum for the filtered value 
    for (int i = 0; i < cant; i++){
      sum += buffer[i]; 
    }

    return sum / cant;
}

// Funciones para actualizar estadísticas
void actualizarEstadisticas() {
  SerialMon.println(F("Actualizar Estadistica"));
    // Actualizar máximos, mínimos y suma para promedios
    // Incrementar contador de mediciones

  if (filtered_value_caudal < min_caudal) {
    min_caudal = filtered_value_caudal;
  } 
  
  if (filtered_value_caudal > max_caudal) {
    max_caudal = filtered_value_caudal;
  }
  
  sum_caudal += filtered_value_caudal;

  if (filtered_value_level < min_level) {
    min_caudal = filtered_value_level;
  } 
  
  if (filtered_value_level > max_level) {
    max_caudal = filtered_value_level;
  }

  sum_level += filtered_value_level;
}

// Funciones para detección de alarmas y errores
void verificarAlarmas() {
  SerialMon.println(F("Verificar Alarmas"));
    // Verificar si los valores de los sensores superan umbrales definidos
    // Actualizar los flags de alarma correspondientemente

  if (filtered_value_caudal < 4.0) {
    flow_meter_disconnected_loop = true;
  }

   if (filtered_value_level < 4.0) {
    level_meter_disconnected_loop = true;
  } 

  if(((filtered_value_level - 4.0) * 100.0 / 16.0) <= MINIMUM_WELL_LEVEL) {
    minimun_level_reached = true;
  }

  if (pump_state == LOW) {
    pump_off = true;
  }
}

void enviarAlarmas() {
  SerialMon.println(F("Enviar Alarmas"));
    // Enviar alarmas al broker MQTT si se han detectado
    // Utilizar gmodem.PublishData() o la función correspondiente

  char payload[32];

  if (flow_meter_disconnected_loop) {
    gModem.publishData("alarma/caudalimetro_desconectado", "ALERTA: se a desconectado el caudalimetro");
  }

  if (level_meter_disconnected_loop) {
    gModem.publishData("alarma/sensor_de_nivel_desconectado", "ALERTA: se a desconectado el sensor de nivel");
  }

  if (pump_off) {
    gModem.publishData("alarma/bomba_apagada", "ALERTA: se a apagado la bomba");
  }
  
  if (minimun_level_reached) {
    snprintf(payload, sizeof(payload), "ALERTA: El nivel del caudal esta por debajo del minimo %4.2f", filtered_value_level);
    gModem.publishData("alarma/minimo_nivel_del_pozo", payload);
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

  char max_caudal_str[] = "Max Caudal";
  char min_caudal_str[] = "Min Caudal";
  char sum_caudal_str[] = "Sum Caudal";
  
  dtostrf(max_caudal, 4, 2, value_str_buffer);
  gModem.publishData(max_caudal_str, value_str_buffer);
  dtostrf(min_caudal, 4, 2, value_str_buffer);
  gModem.publishData(min_caudal_str, value_str_buffer);
  dtostrf(sum_caudal, 4, 2, value_str_buffer);
  gModem.publishData(sum_caudal_str, value_str_buffer);

  char max_level_str[] = "Max Level";
  char min_level_str[] = "Min Level";
  char sum_level_str[] = "Sum Level";

  dtostrf(max_level, 4, 2, value_str_buffer);
  gModem.publishData(max_level_str, value_str_buffer);
  dtostrf(min_level, 4, 2, value_str_buffer);
  gModem.publishData(min_level_str, value_str_buffer);
  dtostrf(sum_level, 4, 2, value_str_buffer);
  gModem.publishData(sum_level_str, value_str_buffer);

  char caudal_count[] = "Contador de pulsos del caudal";
  snprintf(value_str_buffer, sizeof(value_str_buffer), "%ld", caudalPulses);
  gModem.publishData(caudal_count, value_str_buffer);

}

void resetearEstadisticas() {
  SerialMon.println(F("Resetear Estadisticas"));

  for (int i = 0; i < orden_filtro; i++) {
    buffer_sensor_caudal[i] = 0;
    buffer_sensor_level[i] = 0;
  }

  max_caudal = -INFINITY;
  min_caudal = INFINITY;
  sum_caudal = 0.0;

  max_level = -INFINITY;
  min_level = INFINITY;
  sum_level = 0.0;

  contador_mediciones = 0;

  caudalPulses = 0;
  caudalPulseCounter = 0;
}

void resetearAlarmas() {
  SerialMon.println(F("Resetear Alarmas"));

  flow_meter_disconnected_loop = false;
  level_meter_disconnected_loop = false;
  pump_off = false;
  minimun_level_reached = false;

}

// Función de setup
void setup() {

  delay(100);
  pinMode(DI0, INPUT_PULLUP);
  pinMode(DI1, INPUT_PULLUP);
  pinMode(DI2, INPUT_PULLUP);    
  pinMode(DI3, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  SerialMon.println(F("Inicializar comunicación serial para depuración"));
    // Inicializar comunicación serial para depuración
    SerialMon.begin(115200);

    // Configurar pines de entrada para sensores
    // pinMode(PIN_SENSOR1, INPUT);
    // Añadir configuración de pines según sea necesario

  //módem
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

        // Aplicar filtros
        if (orden_filtro == 1) {
          filtered_value_caudal = aplicarFiltroOrden1(caudal, buffer_sensor_caudal[0]);
          filtered_value_level = aplicarFiltroOrden1(level, buffer_sensor_level[0]);
          buffer_sensor_caudal[0] = filtered_value_caudal;
          buffer_sensor_level[0] = filtered_value_level;
        } 
        
        if (orden_filtro == 5) {
          filtered_value_caudal = aplicarMediaMovil(buffer_sensor_caudal, caudal);
          filtered_value_level = aplicarMediaMovil(buffer_sensor_level, level);
        }

        SerialMon.print(F("Valor filtrado de caudal: "));
        SerialMon.println(filtered_value_caudal);
        SerialMon.print(F("Valor filtrado del nivel del pozo: "));
        SerialMon.println(filtered_value_level);

        // Actualizar estadísticas
        // Las estadisticas pueden verse impactadas por el valor filtrado... He adjuntado un pequeño documento al respecto
        actualizarEstadisticas();

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
