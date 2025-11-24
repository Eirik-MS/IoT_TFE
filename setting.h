#include <RIC3D.h>
#include <RIC3DMODEM.h>

// Definiciones de pines y variables globales
#define EMA_FILTER_WEIGTH   0.5 //un numero entre 0 a 1
#define ORDER_FILTER 5

#define SerialMon Serial     // Monitor serial para depuración
#define SerialAT Serial3     // Comunicación con el módem 4G
#define SerialBAUD 115200

#define MAXIMUM_TEMPERATURE 29.0
#define MAX_SPEED_WITH_DOOR_OPEN 5
#define BUS_MAX_CAPACITY 50

extern RIC3DMODEM gModem;

extern int measurement_count;

