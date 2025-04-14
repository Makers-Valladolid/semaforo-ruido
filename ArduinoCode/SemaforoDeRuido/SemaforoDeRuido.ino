/*
 * SemaforoDeRuido
 * 
 * Descripción:
 * Este proyecto implementa un medidor de nivel de ruido ambiental que visualiza los
 * decibelios detectados a través de una tira de LEDs NeoPixel, funcionando como un
 * semáforo de ruido. Los niveles bajos se muestran en verde, los medios en amarillo
 * y los altos en rojo, proporcionando una referencia visual inmediata del nivel sonoro.
 * 
 * Componentes:
 * - Arduino (compatible con AVR)
 * - Micrófono conectado al pin A0
 * - Tira de LEDs NeoPixel conectada al pin 6
 * 
 * Funcionalidad:
 * - Muestrea el sonido ambiente y calcula el nivel en decibelios (dB)
 * - Visualiza el nivel de sonido en la tira de LEDs con escala de colores
 * - Rango calibrado aproximadamente de 40 a 90 dB
 * 
 * Niveles de ruido y su significado:
 * - Verde (Nivel bajo): Por debajo de 70 dB
 *   Seguro para la mayoría de las personas en exposición prolongada.
 * - Amarillo (Nivel moderado): Entre 70 dB y 85 dB
 *   Puede ser molesto y con exposición continua podría empezar a ser perjudicial para la salud.
 * - Rojo (Nivel alto): Por encima de 85 dB
 *   Riesgo de daño auditivo con exposiciones prolongadas. Se recomienda usar protección auditiva en estos niveles.
 */

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// pins
#define MicPin A0
#define LedPin 6

// consts
#define AmpMax (1024/2)
#define MicSamples (512)  // Reducido de 1024*2 a 512 para mayor velocidad
#define USE_3V3 true       // Usar referencia de 3.3V
#define ADC_FLOW true      // Usar ADC en modo flujo
#define DbCalibration 94   // Calibración en decibelios
#define FILTER_FACTOR 0.3  // Factor para filtrado de valores (aumentado para respuesta más rápida)

// Niveles de ruido en decibelios
#define DB_NIVEL_BAJO 70    // Verde
#define DB_NIVEL_MEDIO 85   // Amarillo
// Por encima de DB_NIVEL_MEDIO es rojo

// Definiciones para acceso directo al ADC
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

// Variables globales
int prevdb = 0;
const int maxScale = 12;
Adafruit_NeoPixel pixels(maxScale, LedPin, NEO_GRB + NEO_KHZ800);

// Variable global para el efecto de flash
unsigned long lastFlashTime = 0;
boolean flashState = false;
const int flashInterval = 150; // Intervalo de flash en ms

// Variables para controlar el tiempo de actualización
unsigned long lastUpdateTime = 0;
const int updateInterval = 20;  // Tiempo mínimo entre actualizaciones en ms

// Variables globales adicionales para el peak holding
float peakLevel = 0;          // Nivel de pico actual
float peakDecayRate = 0.08;   // Velocidad de caída del pico (ajustado a un valor intermedio)
unsigned long lastPeakTime = 0;  // Para controlar la caída del pico
const int peakDecayInterval = 75; // Intervalo en ms para actualizar la caída del pico (ajustado)

void setup() {
  // Configuración del ADC
  if (ADC_FLOW) {
    ADCSRA = 0xe0+7;
    ADMUX = 0x0;
    if (!USE_3V3) {
      ADMUX |= 0x40;
    }
    DIDR0 = 0x01;
  } else if (USE_3V3) {
    analogReference(EXTERNAL);
  }

  Serial.begin(9600);
  pixels.begin();
  animation(3);
}

// Función para filtrar los valores de dB
int filtrarDB(int prev, int next) {
  return prev * (1-FILTER_FACTOR) + next * FILTER_FACTOR;
}

// Función para crear color HSV (facilita crear arcoíris)
uint32_t ColorHSV(uint16_t h, uint8_t s, uint8_t v) {
  uint8_t r, g, b;
  
  // Convertir HSV a RGB
  uint8_t region, remainder, p, q, t;
  
  if (s == 0) {
    r = v; g = v; b = v;
    return pixels.Color(r, g, b);
  }
  
  region = h / 43;
  remainder = (h - (region * 43)) * 6; 
  
  p = (v * (255 - s)) >> 8;
  q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
  
  switch (region) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }
  
  return pixels.Color(r, g, b);
}

void loop() {
  // Control de frecuencia de actualización
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime < updateInterval) {
    return;  // Aún no es tiempo de actualizar
  }
  lastUpdateTime = currentTime;
  
  // Medir el nivel de ruido en dB
  int db = MeasureVolume();
  int avgdb = filtrarDB(prevdb, db);
  prevdb = avgdb;  // Guardar el valor filtrado
  
  // Calcular el valor de pico con parte decimal para mayor resolución
  float rangoDb, minDb, maxDb;
  float displayPeakFloat;
  
  if (db < DB_NIVEL_BAJO) {
    // Nivel bajo (verde)
    minDb = 40;
    maxDb = DB_NIVEL_BAJO;
    rangoDb = maxDb - minDb;
    displayPeakFloat = maxScale/3.0 * (db - minDb) / rangoDb;
  } else if (db < DB_NIVEL_MEDIO) {
    // Nivel medio (amarillo)
    minDb = DB_NIVEL_BAJO;
    maxDb = DB_NIVEL_MEDIO;
    rangoDb = maxDb - minDb;
    displayPeakFloat = maxScale/3.0 + (maxScale/3.0) * (db - minDb) / rangoDb;
  } else {
    // Nivel alto (rojo)
    minDb = DB_NIVEL_MEDIO;
    maxDb = 90;
    rangoDb = maxDb - minDb;
    displayPeakFloat = 2*maxScale/3.0 + (maxScale/3.0) * (db - minDb) / rangoDb;
  }
  
  // Limitar el valor a los límites de la escala
  if (displayPeakFloat < 0) displayPeakFloat = 0;
  if (displayPeakFloat > maxScale) displayPeakFloat = maxScale;
  
  // Actualizar el nivel de pico
  if (displayPeakFloat > peakLevel) {
    // Si el nivel actual es mayor que el pico, actualizar inmediatamente
    peakLevel = displayPeakFloat;
    lastPeakTime = currentTime;
  } else if (currentTime - lastPeakTime > peakDecayInterval) {
    // Decrementar el nivel de pico lentamente
    peakLevel -= peakDecayRate;
    if (peakLevel < displayPeakFloat) {
      peakLevel = displayPeakFloat;
    }
    lastPeakTime = currentTime;
  }
  
  // Imprimir valores en serial: db,holdingbar
  Serial.print(db);
  Serial.print(",");
  
  // Convertir el nivel de pico a decibelios para la salida serial
  float peakDb;
  if (peakLevel < maxScale/3.0) {
    // Zona verde
    peakDb = 40 + (DB_NIVEL_BAJO - 40) * peakLevel / (maxScale/3.0);
  } else if (peakLevel < 2*maxScale/3.0) {
    // Zona amarilla
    peakDb = DB_NIVEL_BAJO + (DB_NIVEL_MEDIO - DB_NIVEL_BAJO) * (peakLevel - maxScale/3.0) / (maxScale/3.0);
  } else {
    // Zona roja
    peakDb = DB_NIVEL_MEDIO + (90 - DB_NIVEL_MEDIO) * (peakLevel - 2*maxScale/3.0) / (maxScale/3.0);
  }
  
  Serial.println(peakDb);
  
  // Obtener el índice del LED de pico y su decimal
  int peakLedIndex = (int)peakLevel;
  float peakDecimal = peakLevel - peakLedIndex;
  
  // Asegurar que el índice de peak está dentro del rango válido
  if (peakLedIndex >= maxScale) {
    peakLedIndex = maxScale - 1;
    peakDecimal = 1.0;
  }

  // Verificar si estamos en nivel de alarma (rojo alto)
  boolean isAlarmLevel = (peakDb >= DB_NIVEL_MEDIO);
  
  // Control del efecto de flash para nivel de alarma
  if (isAlarmLevel) {
    // Actualizar estado del flash
    if (currentTime - lastFlashTime > flashInterval) {
      lastFlashTime = currentTime;
      flashState = !flashState;
    }
  } else {
    // Fuera de nivel de alarma, reset del estado de flash
    flashState = false;
  }

  // Determinar el color base según el nivel actual
  uint32_t colorBase;
  if (peakDb >= DB_NIVEL_MEDIO) {
    // Nivel rojo
    colorBase = pixels.Color(255, 0, 0);
  } else if (peakDb >= DB_NIVEL_BAJO) {
    // Nivel amarillo - reducida intensidad para equilibrar con rojo y verde
    colorBase = pixels.Color(180, 180, 0);
  } else {
    // Nivel verde
    colorBase = pixels.Color(0, 255, 0);
  }

  // Actualizar todos los LEDs - Solo basado en el nivel de pico
  for (int i = 0; i < maxScale; i++) {
    uint32_t color;
    
    if (i < peakLedIndex) {
      // LEDs completamente activos
      if (isAlarmLevel && !flashState) {
        // Efecto de flash: cuando está en OFF mostramos un rojo más tenue
        color = pixels.Color(80, 0, 0); // Rojo más tenue durante el flash
      } else {
        color = colorBase;
      }
    } 
    else if (i == peakLedIndex) {
      // LED de transición con brillo proporcional a la parte decimal
      if (peakDb >= DB_NIVEL_MEDIO) {
        // Versión variable del rojo
        if (isAlarmLevel && !flashState) {
          // En flash OFF, usar un rojo más tenue
          color = pixels.Color(peakDecimal * 80, 0, 0);
        } else {
          color = pixels.Color(peakDecimal * 255, 0, 0);
        }
      } else if (peakDb >= DB_NIVEL_BAJO) {
        // Versión variable del amarillo - reducida intensidad
        color = pixels.Color(peakDecimal * 180, peakDecimal * 180, 0);
      } else {
        // Versión variable del verde
        color = pixels.Color(0, peakDecimal * 255, 0);
      }
    }
    else {
      // LED inactivo - baja luminosidad
      if (peakDb >= DB_NIVEL_MEDIO) {
        // Versión tenue del rojo
        color = pixels.Color(2, 0, 0);
      } else if (peakDb >= DB_NIVEL_BAJO) {
        // Versión tenue del amarillo - reducida intensidad
        color = pixels.Color(1, 1, 0);
      } else {
        // Versión tenue del verde
        color = pixels.Color(0, 1, 0);
      }
    }
    
    pixels.setPixelColor(i, color);
  }
  
  pixels.show();
}

void animation(int iteraciones) {
  int sine[] = {0,1,2,3,4,5,6,7,8,9,10,11};
  
  // Hacer efecto arcoíris
  for (int j=0; j<iteraciones; j++) {
    // Hacer dos vueltas completas de colores por cada iteración
    for (int colorStep = 0; colorStep < 360; colorStep += 10) {  // Pasos más pequeños (10 en lugar de 15)
      
      for (int i=0; i<12; i++) {
        // Crear un efecto ondulante con desplazamiento de color
        int hue = (colorStep + i*30) % 360;
        uint32_t color = ColorHSV(hue, 255, 255);
        
        pixels.setPixelColor(sine[i], color);
      }
      
      pixels.show();
      delay(50);  // Delay más largo (50ms en lugar de 20ms)
    }
  }
  
  // Limpieza al final de la animación
  for (int i=0; i<maxScale; i++) {
    pixels.setPixelColor(i, 0);
  }
  pixels.show();
}

float MeasureVolume() {
  long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0;
  
  // Muestrear el micrófono con un método optimizado
  for (int i = 0; i < MicSamples; i++) {
    int k;
    
    // Leer ADC según el modo configurado
    if (ADC_FLOW) {
      while (!(ADCSRA & _BV(ADIF)));  // Esperar a que esté listo el ADC
      sbi(ADCSRA, ADIF);              // Resetear el flag
      k = ADC;                        // Leer el valor directamente (más rápido)
    } else {
      k = analogRead(MicPin);
    }
    
    // Procesar muestra con cálculos optimizados
    int amp = abs(k - AmpMax);
    soundVolRMS += ((long)amp*amp);  // Solo calculamos RMS para decibelios
  }

  // Calcular RMS directamente
  soundVolRMS /= MicSamples;
  float soundVolRMSflt = sqrt(soundVolRMS);

  // Calcular decibelios
  float dB = 20.0*log10(soundVolRMSflt/AmpMax) + DbCalibration;
  
  return dB;
}