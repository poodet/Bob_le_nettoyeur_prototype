/*
*  app/arduino/code_on_esp32/code_on_esp32.ino.cpp
*  SUJET 4 - BOBLE NETTOYEUR
*
*  Created by Jules Graeff, Pauline Odet, Antoine Passemard and Guillaume Bernard.
*  Copyright 2020 CPE Lyon. All rights reserved.
*/

/* BLE libraries */
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

/* Graphics and font library for ST7735 driver chip */
#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

#define LED_PIN 13
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUIDreceive "beb5483e-36e1-4688-b7f5-ea07361b26a8"

/* Global variables */
const int CapteurPin = 2; // broche du capteur infrarouge
const int RELAY_PIN = 27; // broche pour le signal allant sur le relay de la pompe
int capteur_state; // etat de la sortie du capteur
String valor;
bool depassementFlag = false; // flag mis a true quand on a eu un dépassement dans le vide

class ReceiveBLECallback: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string value = pCharacteristic->getValue();
     
      if (value.length() > 0) {

        valor = "";
        for (int i = 0; i < value.length(); i++)
        {
          if (value[i] != 0)  // Remove \x00 of the received BLE msg
          {
            valor = valor + value[i];
          }
        }

        if(valor == "BLE/spray")
        {
          digitalWrite(RELAY_PIN, HIGH); // turn on pump
          delay(2000);
          digitalWrite(RELAY_PIN, LOW);  // turn off pump
          delay(2000);
        }
        
        Serial.println(valor);
      }    
    }
};

void setup() {

  Serial.begin(115200);

  BLEDevice::init("BOB_LE_NETTOYEUR");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUIDreceive,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new ReceiveBLECallback());
  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  pinMode(CapteurPin, INPUT); //la broche du capteur est mise en entree

  Serial.begin(115200);

  // Initialize digital pin 27 as an output.
  pinMode(RELAY_PIN, OUTPUT);
  
  tft.init();
  tft.setRotation(1);
  tft.setTextSize(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLUE, TFT_BLACK);

  tft.drawString("Bonjour, je suis Bob Le Nettoyeur !", 0, 0, 2);

  delay(3000);
  tft.fillScreen(TFT_BLACK); 
}

void loop()
{
  capteur_state = digitalRead(CapteurPin); //lecture du capteur

  tft.fillScreen(TFT_BLACK);
  
  if (capteur_state == LOW) // si la table est detectee
  {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("Table", 0, 0, 2);
      
      Serial.println("CAPTEUR/table");
  }

  else // rien n'est detecté dans les 5 cm == VIDE
  {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("VIDE !", 0, 0, 2);
    
    // Print on serial BUS "vide". Mettre un flag à true.
    Serial.println("CAPTEUR/vide");
    depassementFlag = true;
  }
}
