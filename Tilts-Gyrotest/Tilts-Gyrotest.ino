#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int pinLED = 13;

float sensitivity = 50.0;

const int calibrationSamples = 200;
float totalAngularVelocity = 0.0;

const byte tP2 = 2;   // tP significando tiltPin
const byte tP3 = 3;
const byte tP4 = 4;
const byte tP5 = 5;
const byte tP6 = 6;
const byte tP7 = 7;

int tVal2, tVal3, tVal4, tVal5, tVal6, tVal7, ShownFaceValue; // TVal significando TiltValue

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  calibrateGyro();
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, LOW);

  pinMode(tP2, INPUT_PULLUP);    //Obtencion de señales de los tilts
  pinMode(tP3, INPUT_PULLUP);
  pinMode(tP4, INPUT_PULLUP);
  pinMode(tP5, INPUT_PULLUP);
  pinMode(tP6, INPUT_PULLUP);
  pinMode(tP7, INPUT_PULLUP);

}

void loop() {

  tVal2 = digitalRead(tP2);    //Señal de pin a valores de pin
  tVal3 = digitalRead(tP3);
  tVal4 = digitalRead(tP4);
  tVal5 = digitalRead(tP5);
  tVal6 = digitalRead(tP6);
  tVal7 = digitalRead(tP7);

  int16_t accX, accY, accZ;
  int16_t gyroX, gyroY, gyroZ;
  
  mpu.getAcceleration(&accX, &accY, &accZ);
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  
  // Convertir los valores de aceleración y velocidad angular a unidades reales
  float ax = (float)accX / 16384.0; // Dividir por 16384 para obtener g (gravedad, 9.81 m/s^2)
  float ay = (float)accY / 16384.0;
  float az = (float)accZ / 16384.0;
  
  float gx = (float)gyroX / sensitivity; // Convertir a grados por segundo
  float gy = (float)gyroY / sensitivity;
  float gz = (float)gyroZ / sensitivity;
  
  // Calcular los ángulos de inclinación en grados
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  
  // Asegurarnos de que los ángulos estén entre 0 y 360 grados
  pitch = (pitch < 0) ? (pitch + 360) : pitch;
  roll = (roll < 0) ? (roll + 360) : roll;
  
  /*// Mostrar los ángulos en el monitor serial, en grados
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("  ");
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("  ");
  */
  // Si la velocidad angular supera un umbral, encender el LED
  /*
  if (gz > 5.0 || gz < -5.0) {
    digitalWrite(pinLED, HIGH);
    Serial.print("Movimiento detectado");
  } else {
    digitalWrite(pinLED, LOW);
  }
  */
    // Identificar la cara correspondiente según el ángulo de inclinación
  int face;



  if ((pitch >= 0 && pitch < 20) && (roll >= 230 && roll < 360) && tVal2 == 0) {
    face = 1;
  } else if ((pitch >= 350 && pitch < 360) && (roll >= 40 && roll <85 ) && tVal3 == 0 ) {
    face = 2;
  } else if ((pitch >= 340 || (pitch >= 357 && pitch < 350)) && (roll >= 270 && roll < 280) && tVal4 == 0 ){
    face = 3;
  } else if ((pitch >= 50 && pitch < 90) && (roll >= 240 && roll < 360) && tVal5 == 0 ){
    face = 4;
  } else if ((pitch >= 270 && pitch < 300) && (roll >= 0 && roll < 360) && tVal6 == 0  ) {
    face = 5;
  } else if ((pitch >= 350 && pitch < 360) && (roll >= 350 || (roll >= 350 && roll < 360)) && tVal7 == 0 ) {     // cara 1
    face = 6;
  }else {
    face = 0;   // Cara 0 para no identificado
  }

  
  //Muestra valores de tilts. el orden es el tilt.

  Serial.print(tVal2);  //tilt 1
  Serial.print(",");
  Serial.print(tVal3);  //tilt 2
   Serial.print(",");
  Serial.print(tVal4);  //tilt 3
   Serial.print(",");
  Serial.print(tVal5);  //tilt 4
   Serial.print(",");
  Serial.print(tVal6);  //tilt 5
   Serial.print(",");
  Serial.print(tVal7);  //tilt 6

   Serial.print(",");
  //Serial.println("  ---------  Cara Mostrada (tilts):");
  //Serial.println(ShownFaceValue);
  
  //Muestra X , Y, Z tanto del giroscopio como del acelerometro

  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);

  Serial.print(",");

   Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);

  // Mostrar la cara identificada
  Serial.print(",");
  Serial.println(face);
  delay(100); // Ajusta este valor según la frecuencia de actualización deseada
}

void calibrateGyro() {
  Serial.println("Calibrando giroscopio...");
  
  for (int i = 0; i < calibrationSamples; i++) {
    int16_t gyroX, gyroY, gyroZ;
    mpu.getRotation(&gyroX, &gyroY, &gyroZ);
    totalAngularVelocity += gyroZ / sensitivity;
    delay(10);
  }
  
  float averageAngularVelocity = totalAngularVelocity / calibrationSamples;
  sensitivity = averageAngularVelocity;
  
  Serial.print("Calibración completa. Sensibilidad: ");
  Serial.println(sensitivity);
}
