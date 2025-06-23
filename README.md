# yaqueline
Reciclage de PETG para filamento 3D


# ♻️ PET Filament Recycler con Arduino Nano

Este proyecto implementa una máquina basada en **Arduino Nano** que convierte tiras de envases PET reciclados en filamento para impresoras 3D. El sistema controla una bobina calefactora y un motor paso a paso para extruir el material fundido.

## 🚀 Objetivo

Diseñar e implementar una máquina de bajo costo que transforme botellas PET en filamento 3D reutilizable, impulsando la economía circular y la fabricación sustentable.

## 🧩 Componentes del sistema

- **Arduino Nano** – Microcontrolador principal.
- **Motor paso a paso + driver ULN2003 o A4988** – Para mover el mecanismo de extrusión.
- **Sensor de temperatura (ej. MAX31855 + termocupla tipo K)** – Monitorea la temperatura de fusión.
- **Cartucho calefactor o resistencia tipo cartucho** – Funde el PET.
- **Relé de estado sólido (SSR)** – Controla el encendido de la bobina.
- **Fuente de poder** – Suministro adecuado para motor, calefactor y Arduino.
- **Botones o potenciómetros (opcional)** – Control manual.
- **Pantalla LCD 16x2 (opcional)** – Para mostrar temperatura y estado.

## 🛠️ Instalación

### Requisitos

- [Arduino IDE](https://www.arduino.cc/en/software)
- Bibliotecas:
  - `Stepper.h`
  - `Adafruit_MAX31855.h` (instalable desde el Library Manager)

### Conexiones sugeridas

| Componente         | Pin Arduino Nano |
|--------------------|------------------|
| Motor paso a paso  | 8, 9, 10, 11      |
| MAX31855 (SPI)     | 10 (CS), 12 (MISO), 13 (SCK) |
| Cartucho calefactor| SSR controlado desde D7 |

> ⚠️ Asegúrate de usar una fuente de energía independiente para el calefactor y el motor. El Arduino debe estar aislado de cargas de alto voltaje.

## 🧪 Código de prueba

El archivo `filament_recycler.ino` contiene un ejemplo inicial que activa el motor paso a paso cuando la temperatura supera los **200 °C**.

```cpp
#include <Stepper.h>
#include <Adafruit_MAX31855.h>

const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

#define MAX31855_CS 10
#define SSR_PIN 7

Adafruit_MAX31855 thermocouple(MAX31855_CS);

void setup() {
  Serial.begin(9600);
  myStepper.setSpeed(100);
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);
}

void loop() {
  double temperature = thermocouple.readCelsius();
  Serial.print("Temperatura: ");
  Serial.println(temperature);

  if (temperature >= 200) {
    digitalWrite(SSR_PIN, HIGH); // Enciende calefactor
    myStepper.step(stepsPerRevolution); // Extrusión
  } else {
    digitalWrite(SSR_PIN, LOW); // Apaga calefactor
  }

  delay(1000);
}
