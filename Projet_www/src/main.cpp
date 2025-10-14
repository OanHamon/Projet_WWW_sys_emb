#include <ChainableLED.h> // https://github.com/pjpmarques/ChainableLED
#include <Wire.h>
#include "DS1307.h" // https://github.com/Seeed-Studio/RTC_DS1307
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> // https://github.com/adafruit/Adafruit_BME280_Library
#include <SoftwareSerial.h>

// GPS sur broches 4 (RX) et 5 (TX)
SoftwareSerial gpsSerial(4, 5); // RX, TX

// Définir les ports utilisés
const int buttonPinRouge = 2;
const int buttonPinVert = 3;

DS1307 clock;
ChainableLED led(7, 8, 1);
Adafruit_BME280 bme; // Utilisation en I2C

void printTime() {
  clock.getTime();
  Serial.print(clock.hour, DEC); Serial.print(":");
  Serial.print(clock.minute, DEC); Serial.print(":");
  Serial.print(clock.second, DEC); Serial.print("  ");
  Serial.print(clock.month, DEC); Serial.print("/");
  Serial.print(clock.dayOfMonth, DEC); Serial.print("/");
  Serial.print(clock.year + 2000, DEC); Serial.println("");
}

void setupTime() {
  clock.fillByYMD(2025, 10, 8);
  clock.fillByHMS(15, 59, 00);
  clock.setTime();
}

void setup() {
  pinMode(buttonPinRouge, INPUT);
  pinMode(buttonPinVert, INPUT);
  Serial.begin(9600);
  gpsSerial.begin(9600); // Initialisation GPS
  led.setColorRGB(0, 0, 0, 0);
  clock.begin();
  setupTime();

  // Initialisation du capteur BME280
  if (!bme.begin(0x76)) {
    Serial.println("Erreur : capteur BME280 non détecté !");
    while (1);
  }
}

String Lecture_GPS(){
  String gpsData = "";
  if (gpsSerial.available()) {
    bool t = true;
    while (t) {
      gpsData = gpsSerial.readStringUntil('\n');
      if (gpsData.startsWith("$GPGGA")) {
        t = false;
      }
    }
    return gpsData;
    /*
$GPGGA : Type de trame (Global Positioning System Fix Data).
120937.000 : Heure UTC (12:09:37.000).
4716.6952,N : Latitude (47° 16.6952' Nord).
00212.7405,W : Longitude (2° 12.7405' Ouest).
1 : Qualité du signal GPS :
  0 = pas de position
  1 = GPS fixe
  2 = DGPS fixe
9 : Nombre de satellites utilisés.
1.01 : Précision horizontale (HDOP).
-4.4,M : Altitude au-dessus du niveau de la mer (-4.4 mètres).
49.1,M : Hauteur du géoïde (différence entre le niveau de la mer et l’ellipsoïde WGS84).
Champ vide : Temps de correction DGPS (non utilisé ici).
Champ vide : ID de station DGPS (non utilisé ici).
*63 : Somme de contrôle (checksum).*/
  }
}

void loop() {
  // Lecture des boutons
  int buttonStateRouge = digitalRead(buttonPinRouge);
  int buttonStateVert = digitalRead(buttonPinVert);
  led.setColorRGB(0, 255 * (-(buttonStateRouge - 1)), 255 * (-(buttonStateVert - 1)), 0);

  // Capteur de lumière
  int lightSensor = analogRead(A0);

  // Affichage des états
  Serial.print("Bouton Rouge : "); Serial.println(buttonStateRouge);
  Serial.print("Bouton Vert : "); Serial.println(buttonStateVert);
  Serial.print("Lumière : "); Serial.println(lightSensor);

  // Capteur BME280
  Serial.print("Température : "); Serial.print(bme.readTemperature()); Serial.println(" °C");
  Serial.print("Humidité : "); Serial.print(bme.readHumidity()); Serial.println(" %");
  Serial.print("Pression : "); Serial.print(bme.readPressure() / 100.0F); Serial.println(" hPa");

  // Lecture GPS : attend une phrase GPGGA
  Serial.println(Lecture_GPS());

  // Affichage de l'heure
  printTime();

  delay(1000);
}
