struct Posicion {
    float lat;
    float lon;
};

float aplicarFiltroOrden1(float valor_actual, float valor_anterior);

float aplicarMediaMovil(float* buffer, float nuevo_valor);

float mA_to_temp(float mA);

int generarNumeroAleatorio(int min, int max);

float generarNumeroAleatorioDecimal(float min, float max);

float calcularDemora(float velocidad);

void calcularTiemposEspera(int indiceActual, float velocidad, float tiempos[]);
