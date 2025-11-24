
#include <Arduino.h>
#include "utility.h"
#include "setting.h"

// Funciones para aplicar filtros
float aplicarFiltroOrden1(float valor_actual, float valor_anterior) {
    SerialMon.println(F("Usar filtro de Primer Orden"));
    return valor_actual * EMA_FILTER_WEIGTH + valor_anterior * (1 - EMA_FILTER_WEIGTH);
}

float aplicarMediaMovil(float* buffer, float nuevo_valor) {
    SerialMon.println(F("Filtro Media Movil"));
    float sum = 0; 

    //shift values in the buffer first
    for (int j = ORDER_FILTER - 1; j > 0; j--){
      buffer[j] = buffer[j-1];
    }
    buffer[0] = nuevo_valor;

    int cant = measurement_count < ORDER_FILTER ? measurement_count : ORDER_FILTER;
    //calculate the sum for the filtered value 
    for (int i = 0; i < cant; i++){
      sum += buffer[i]; 
    }

    return sum / cant;
}

float mA_to_temp(float mA){
    return 5*(mA-4) - 20;
}
