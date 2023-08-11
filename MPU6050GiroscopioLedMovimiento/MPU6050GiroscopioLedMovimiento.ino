#include <Wire.h>
#include <MPU6050.h>

// Inicializar el objeto MPU6050
MPU6050 mpu;

// Definir el pin del LED
const int pinLED = 13;

// Definir la sensibilidad del giroscopio
float sensitivity = 131.0;

// Variables para la calibración
int calibrationSamples = 200;
float totalAngularVelocity = 0.0;

void setup() {
  // Iniciar la comunicación serie
  Serial.begin(9600);
  
  // Iniciar la comunicación I2C
  Wire.begin();
  
  // Inicializar el MPU6050
  mpu.initialize();
  
  // Calibrar el giroscopio
  calibrateGyro();
  
  // Encender el LED
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, LOW);
}

void loop() {
  // Leer los valores del giroscopio
  int16_t gyroX, gyroY, gyroZ;
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  
  // Calcular la velocidad angular en el eje Z
  float angularVelocity = gyroZ / sensitivity;
  
  // Si la velocidad angular supera un umbral, encender el LED
  if (angularVelocity > 5.0 || angularVelocity < -5.0) {
    digitalWrite(pinLED, HIGH);
    Serial.println("Movimiento detectado");
  } else {
    digitalWrite(pinLED, LOW);
  }
  
  // Esperar un breve periodo de tiempo
  delay(100);
}

void calibrateGyro() {
  Serial.println("Calibrando giroscopio...");
  
  // Tomar muestras y sumar los valores de velocidad angular
  for (int i = 0; i < calibrationSamples; i++) {
    int16_t gyroX, gyroY, gyroZ;
    mpu.getRotation(&gyroX, &gyroY, &gyroZ);
    totalAngularVelocity += gyroZ;
    delay(10);
  }
  
  // Calcular el valor promedio de velocidad angular
  float averageAngularVelocity = totalAngularVelocity / calibrationSamples;
  
  // Actualizar la sensibilidad del giroscopio para la calibración
  sensitivity = averageAngularVelocity;
  
  Serial.print("Calibración completa. Sensibilidad: ");
  Serial.println(sensitivity);
}
