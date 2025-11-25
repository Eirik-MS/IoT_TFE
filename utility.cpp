
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

int generarNumeroAleatorio(int min, int max) {
    if (min > max) return min;
    return random(min, max + 1);
}

float generarNumeroAleatorioDecimal(float min, float max) {
    if (min > max) return min;
    float r = min + (random(0, 10001) / 10000.0) * (max - min);
    return round(r * 10.0) / 10.0;
}

float calcularDemora(float velocidad) {
    float demoraBase = 12.0 - ((velocidad - 10.0) * (8.0 / 20.0));

    float ruido = generarNumeroAleatorio(-5, 5) / 10.0;

    float demoraFinal = demoraBase + ruido;

    if (demoraFinal < 4.0) demoraFinal = 4.0;
    if (demoraFinal > 12.0) demoraFinal = 12.0;

    return demoraFinal;
}

void calcularTiemposEspera(int indiceActual, float velocidad, float tiempos[]) {
    float acumulado = 0;

    for (int i = 0; i < totalPosiciones; i++) {
        if (i <= indiceActual) {
            tiempos[i] = 0;
        } else {
            float demora = calcularDemora(velocidad);
            acumulado += demora;
            tiempos[i] = acumulado;
        }
    }
}
