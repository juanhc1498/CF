//Librerias  https://www.youtube.com/watch?v=J2Mae4LPj-E  https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

//variables
int Pulsador = 7;
int Pulsador2 = 4;
int Pulsador3 = 3;
#define PinEcho 6 
#define PinTrig 5
                                  
SoftwareSerial mySoftwareSerial(10, 11);
DFRobotDFPlayerMini myDFPlayer;

//sensor
long duracion, distancia;

void setup()
{
  mySoftwareSerial.begin(9600);
  //Serial.begin(115200);// codigo de comunicacion entre arduino y pc (se puede quitar porque no estara cnectado a pc)
  pinMode(8,INPUT);//BUSY
  pinMode(Pulsador,INPUT);
  pinMode(PinEcho, INPUT);//SENSOR
  pinMode(PinTrig, OUTPUT);//SENSOR

   //verificacion de conec¿xion
  if (!myDFPlayer.begin(mySoftwareSerial))//Use softwareSerial to communicate with mp3.
    {
    Serial.println("Error conexiones");
    while(true);
  }
  Serial.println("Conexion exitosa");

  myDFPlayer.volume(30);  //Set volume value (0~30)

}

void loop()
{
  static unsigned long timer = millis();
  boolean PinBusy = digitalRead(8); //muestra el estado del modulo(si reproduce emite 0V, si no 5V)
                                  //usamos el pin 8 como entrada
  //sensor
  digitalWrite(PinTrig, LOW);//el pulso empieza apagado
  delayMicroseconds(2);
  digitalWrite(PinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(PinTrig, LOW);

  duracion = pulseIn(PinEcho, HIGH);//medimos la duracion del pulso encendido
  distancia = (duracion/2)/29;//calulamos la distancia, (duracion/2) ida y vuelta
                              //29 porque es la vel sonido en cm

  if (distancia >= 20 && distancia <= 60)
   {
     /* if (millis() - timer > 3000) {
    timer = millis();
    myDFPlayer.next();
    myDFPlayer.play(1);
    PinBusy == true;
      }*/
      
     PinBusy == true;
     Serial.println("S1");
     myDFPlayer.play(1);    
     Serial.print(distancia); //envio el valor de la distancia
     Serial.println("cm");
     delay(4000);
     }

   if (distancia >= 61 && distancia <= 110)
   {
     PinBusy == true;
     Serial.println("S2");
     myDFPlayer.play(2);  //Play the first mp3   
     Serial.print(distancia); //envio el valor de la distancia
     Serial.println("cm");
     delay(4000);
   }

   if (distancia >= 111 && distancia <= 160)
   {
     PinBusy == true;
     Serial.println("S3");
     myDFPlayer.play(3);     
     Serial.print(distancia); //envio el valor de la distancia
     Serial.println("cm"); 
     delay(10000);   
   }   

   //prueba tiempo ---------------------------------------------------
/*static unsigned long timer = millis();

  if (millis() - timer > 3000) {
    timer = millis();
    myDFPlayer.next();  //Play next mp3 every 3 second.
  }*/
   //fin prueba tiempo ---------------------------------------------------

//pausa
  if (distancia >= 10 && distancia <=20)
    {
     PinBusy == true;
     Serial.println("Pausa");
     myDFPlayer.pause();
     Serial.print(distancia); //envio el valor de la distancia
     Serial.println("cm"); 
    }   

  //prueba pulsador-----------------------------
  
  if(digitalRead(Pulsador) == LOW)
    {
      PinBusy == true;
      Serial.println("P4");
      myDFPlayer.play(4);
      delay(10000);
    }

    if(digitalRead(Pulsador2) == LOW)
    {
      PinBusy == true;
      Serial.println("P5");
      myDFPlayer.play(5);
      delay(10000);
    }

    if(digitalRead(Pulsador3) == LOW)
    {
      PinBusy == true;
      Serial.println("P6");
      myDFPlayer.play(6);
      delay(12000);
    }

 }
  
