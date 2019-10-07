//librerias
//----conexion
#include "WiFi.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>//me da el tiempo(fecha y hora)
#include <WiFiUdp.h>

//-----ky001 (dallas)
#include <OneWire.h>
#include <DallasTemperature.h>

//DHT 11
#include "DHT.h"

//IMU
#include "MPU9250.h"

//--------------------------variables-------------------------------------------------

//Buzzer
int frecuencia = 2000;
int canal = 0;
int resolucion = 8;


//LEDS
const int LedTC = 25;//led temperatura corporal
const int LedAC = 33;//led acelererometro

//ky001 (dallas)
#define SensorDallas 26 //pin
OneWire oneWire(SensorDallas);
DallasTemperature BibliotecaSensorDallas(&oneWire);//inicializa la biblioteca

//GSR (Vcc -> 5v, A2 -> Sig, GND -> GND)
#define SensorGSR 4 //pin
int threshold = 0;
int EstadoSensor = 0;

//DHT 11
uint8_t SensorDHT = 27; //pin
#define TipoDHT DHT11
float ta = 0;//tempertura ambiente
float ha = 0;//humedad ambiente
DHT dht(SensorDHT, TipoDHT);

//----MPU9250 (aceletrometro)
MPU9250 IMU(Wire,0x68);
int status;

float ax,ay,az;
float gx, gy, gz;
float mx, my, mz;
float temperaturaCircuito;

//conexion wifi
const char * ID_Red = "FAMILIA_HURTADO";//"WiFi-UAO"; //nombre de la red wifi
const char * Password_Red = "juan1998"; //contraseña de la red si tiene se ingresa entre "", si no solo se deja ""

//tamaño de los datos del json
const size_t tamanioJSON = JSON_OBJECT_SIZE(2) + 21 * JSON_OBJECT_SIZE(3) + 2 * JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(10);
int intentosConexion = 0;//intentos para conectarse a la red

//definir cliente NTP para el tiempo
WiFiUDP NTP_UDP;//Network Time Protocol Protocolo de datagramas de usuario
NTPClient tiempoClient(NTP_UDP);//Network Time Protocol Client

//variables fecha y hora
String formato;
String fecha;
String hora;

//---------------------------------------------------------------------------------------SETUP--------------------------------------------------------------------------
void setup(){
  
 Serial.begin(9600);

  ledcSetup(canal, frecuencia, resolucion);
  ledcAttachPin(2, canal);

  pinMode(LedTC, OUTPUT);
  pinMode(LedAC, OUTPUT);
 
 // Iniciar la biblioteca
 BibliotecaSensorDallas.begin(); // Establece valor por defecto del chip 9 bit. Si tiene problemas pruebe aumentar
 dht.begin();

 BuscarWifi();

 //si se conecta en menos de 15 intentos nos da la comprobacion y la ip
 if(intentosConexion > 30){
  Serial.println("");
  Serial.println("conectado a wifi");
  Serial.println(WiFi.localIP());
 }

  //si no se conecta lo intentara de nuevo
  else{
        Serial.println("");
        Serial.println("wifi error");

        while(intentosConexion > 30){        
            
            BuscarWifi();
            
          }//fin while
          
    }//fin else

  tiempoClient.begin();
  tiempoClient.setTimeOffset(-18000);/*esto es por la zona horaria, dependiendo en que pais
                                     estes cambiara el numero*/


//iniciamos comunicacion con la IMU
status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}

  //opciones de acelerometro escala completa rango +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);

  //opciones de giroscopio escala completa rango +/- 500 grados/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);

  //setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

  //setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
  }
                                     
}//fin setup

//------------------------------------------------------------------------------------fin Setup--------------------------------------------------------------------------

//------------------------------------------------------------------------------------LOOP--------------------------------------------------------------------------------
void loop(){
   
   if(WiFi.status() == WL_CONNECTED){

    DatosDHT();
    DatosGSR();
    DatosDallas();
    DatosMPU();

      if(isnan(ha) || isnan(ta)){
        Serial.println("Error al leer del sensor DHT");
        return;
        }

        //resibimos un string que es array de chars que tambien es un objeto json a traves del metodo devolver_json
 String info = devolver_json(
  37.2, //body temperature value
  DatosGSR(), //body resistenace value
  70, //body BPM value
  95.4, //body sp02 value
  ta, //valor temperatura ambiente
  ha, //valor humedad ambiente
  ax,//device acceleration ax
  ay,//device acceleration ay
  az,//device acceleration az
  gx,//device gyro gx
  gy,//device gyro gy
  gz,//device gyro gz
  mx,//device magnetometer mx
  my,//device magnetometer my
  mz,//device magnetometer mz
  234.5,//device pressure value
  950.3,//device altitude value
  temperaturaCircuito);//device temperature value 
   
//le enviamos los datos al servidor 
EnvioDatosCliente(info);
  } 
  
  else{
    Serial.println("Error in WiFi connection");  
    Serial.println("Reeconectando:");  
    BuscarWifi();
    }
      
 }//fin loop*/
//------------------------------------------------------------------------------------fin LOOP--------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------BUZZER--------------------------------------------------------------------------
void Buzzer(){
   
   ledcWriteTone(canal, 500);
   delay(50);//1000
   
 }

void BuzzerOff(){
   
   ledcWriteTone(canal, 0);
   delay(50);//1000
   
 }
//------------------------------------------------------------------------------------fin BUZZER------------------------------------------------------------------------

//-----------------------------------------------------------------------------------DatosDallas-------------------------------------------------------------------------
void DatosDallas() {
 // llamar a Bibliotecasensor.requestTemperatures() para emitir
 // un pedido de temperatura global a todos los que esten en el bus
 BibliotecaSensorDallas.requestTemperatures();  // Enviar el comando para obtener la temperatura
 //Serial.println("Midiendo...");
 Serial.print("temperatura corporal es: ");
 Serial.print(BibliotecaSensorDallas.getTempCByIndex(0)); 
 // uso de getTempCByIndex para obtener mas datos
 Serial.println(" ");

 
if(BibliotecaSensorDallas.getTempCByIndex(0)>= 37.8){

     digitalWrite(LedTC, HIGH);
     delay(50);//1000
 
 }else{digitalWrite(LedTC, LOW);
     delay(50);//1000
     }
 
}//fin DatosDallas

//------------------------------------------------------------------fin DatosDallas-------------------------------------------------------------------------------------

//-------------------------------------------------------------------DatosGSR------------------------------------------------------------------------------------------------
int DatosGSR() {

  int datos = 0;
  long sum = 0;
  for(int i=0; i<300; i++)
    {
     EstadoSensor = analogRead(SensorGSR);
     sum += EstadoSensor;
     delay(5); 
     }
     
   threshold = sum/300;
   datos = ((2048+2 * threshold)*10000)/(1024-threshold);
   Serial.print("GSR: ");
   Serial.println(datos);
   Serial.println("");
   delay(50);//100


  return datos;
   
  }//fin DatosGSR

//------------------------------------------------------------------------------fin DatosGSR---------------------------------------------------------------------------

//-------------------------------------------------------------------------------DatosDHT------------------------------------------------------------------------------
void DatosDHT() {
    ta = dht.readTemperature();
    ha = dht.readHumidity();
    Serial.print("temperatura ambiente: ");
    Serial.println(ta);
    Serial.print("humedad ambiente: ");
    Serial.println(ha);
    
  }//fin DatosDHT

//-----------------------------------------------------------------------------------------fin DatosDHT------------------------------------------------------------------

//------------------------------------------------------------------------------------DatosMPU---------------------------------------------------------------------------
void DatosMPU(){
  
  //leo el sensor
  IMU.readSensor();

  //aceleracion
  Serial.print("aceleracion en X es: ");
  ax = IMU.getAccelX_mss();
  Serial.println(ax);
  Serial.print("");

  Serial.print("aceleracion en Y es: ");
  ay = IMU.getAccelY_mss();
  Serial.println(ay);
  Serial.print("");

  Serial.print("aceleracion en Z es: ");
  az = IMU.getAccelZ_mss();
  Serial.println(az);
  Serial.println("");
  

  //Giros
  Serial.print("el giro en X es: ");
  gx = IMU.getGyroX_rads();
  Serial.println(gx);
  Serial.print("");
  
  Serial.print("el giro en Y es: ");
  gy = IMU.getGyroY_rads();
  Serial.println(gy);
  Serial.print("");
  
  Serial.print("el giro en Z es: ");
  gz = IMU.getGyroZ_rads();
  Serial.println(gz);
  Serial.print(""); 

//campo magnetico
  Serial.print("Magnetismo en X es: ");
  mx = IMU.getMagX_uT();
  Serial.println(mx);
  Serial.print("");

  
  Serial.print("Magnetismo en Y es: ");
  my = IMU.getMagY_uT();
  Serial.println(my);
  Serial.print("");

  
  Serial.print("Magnetismo en Z es: ");
  mz = IMU.getMagZ_uT();
  Serial.println(mz);
  Serial.print("");

  //sensor de teperatura
  Serial.print("la temperatura del circuito es: ");
  temperaturaCircuito = IMU.getTemperature_C();
  Serial.println(temperaturaCircuito);
  delay(50);//2000


if(IMU.getAccelY_mss() >= 50){

     Buzzer();
     digitalWrite(LedAC, HIGH);
     delay(50);//1000
 
 }else{digitalWrite(LedAC, LOW);
     BuzzerOff();
     delay(50);//1000
     }
  
  }

//---------------------------------------------------------------- fin DatosMPU ---------------------------------------------------------


//------------------------------BuscarWifi------------------------------------------------------
void BuscarWifi(){

  WiFi.begin(ID_Red,  Password_Red);//manda el nombre de la red wifi y la contraseña

  //cuenta hasta 30, si no se puede conectar lo cancela
  while(WiFi.status() != WL_CONNECTED and intentosConexion < 30 )
    {
      intentosConexion ++;
      delay(500);
      Serial.print("Buscando..");
    }

  if(WiFi.status() == WL_CONNECTED){

        intentosConexion == 0;
        Serial.println("conectado a WIFI");
        Serial.println(WiFi.localIP());    
      }
   
  }//fin BuscarWifi

//-------------------------------------------------------------------fin BuscarWifi---------------------------------------------------------------------------

//-----------------------------------------------------mensajes JSON------------------------------------------------------------------------------------------
String devolver_json(float v1, int v2, int v3, float v4, float v5, float v6, int v7, int v8, int v9, float v10, float v11,
float v12, int v13, int v14, int v15, float v16, float v17,float v18){
  
DynamicJsonDocument doc(tamanioJSON);

//le mandamos la fecha y la hora del metodo tiempo_actual
doc["time"] = tiempo();

JsonObject body = doc.createNestedObject("body");

JsonObject body_temperature = body.createNestedObject("temperature");
body_temperature["value"] = v1;
body_temperature["type"] = "float";
body_temperature["unit"] = "°C";

JsonObject body_resistance = body.createNestedObject("resistance");
body_resistance["value"] = v2;
body_resistance["type"] = "int";
body_resistance["unit"] = "Ohm";

JsonObject body_BPM = body.createNestedObject("BPM");
body_BPM["value"] = v3;
body_BPM["type"] = "int";
body_BPM["unit"] = "Dimensionless";

JsonObject body_SpO2 = body.createNestedObject("SpO2");
body_SpO2["value"] = v4;
body_SpO2["type"] = "float";
body_SpO2["unit"] = "%";

JsonObject ambient = doc.createNestedObject("ambient");

JsonObject ambient_temperature = ambient.createNestedObject("temperature");
ambient_temperature["value"] = v5;
ambient_temperature["type"] = "float";
ambient_temperature["unit"] = "°C";

JsonObject ambient_humidity = ambient.createNestedObject("humidity");
ambient_humidity["value"] = v6;
ambient_humidity["type"] = "float";
ambient_humidity["unit"] = "%";

JsonObject device = doc.createNestedObject("device");
device["UUID"] = "550e8400-e29b-41d4-a716-446655440000";

JsonObject device_acceleration = device.createNestedObject("acceleration");

JsonObject device_acceleration_ax = device_acceleration.createNestedObject("ax");
device_acceleration_ax["value"] = v7;
device_acceleration_ax["type"] = "int";
device_acceleration_ax["unit"] = "mg";

JsonObject device_acceleration_ay = device_acceleration.createNestedObject("ay");
device_acceleration_ay["value"] = v8;
device_acceleration_ay["type"] = "int";
device_acceleration_ay["unit"] = "mg";

JsonObject device_acceleration_az = device_acceleration.createNestedObject("az");
device_acceleration_az["value"] = v9;
device_acceleration_az["type"] = "int";
device_acceleration_az["unit"] = "mg";

JsonObject device_gyro = device.createNestedObject("gyro");

JsonObject device_gyro_gx = device_gyro.createNestedObject("gx");
device_gyro_gx["value"] = v10;
device_gyro_gx["type"] = "float";
device_gyro_gx["unit"] = "deg/s";

JsonObject device_gyro_gy = device_gyro.createNestedObject("gy");
device_gyro_gy["value"] = v11;
device_gyro_gy["type"] = "float";
device_gyro_gy["unit"] = "deg/s";

JsonObject device_gyro_gz = device_gyro.createNestedObject("gz");
device_gyro_gz["value"] = v12;
device_gyro_gz["type"] = "float";
device_gyro_gz["unit"] = "deg/s";

JsonObject device_magnetometer = device.createNestedObject("magnetometer");

JsonObject device_magnetometer_mx = device_magnetometer.createNestedObject("mx");
device_magnetometer_mx["value"] = v13;
device_magnetometer_mx["type"] = "int";
device_magnetometer_mx["unit"] = "mG";

JsonObject device_magnetometer_my = device_magnetometer.createNestedObject("my");
device_magnetometer_my["value"] = v14;
device_magnetometer_my["type"] = "int";
device_magnetometer_my["unit"] = "mG";

JsonObject device_magnetometer_mz = device_magnetometer.createNestedObject("mz");
device_magnetometer_mz["value"] = v15;
device_magnetometer_mz["type"] = "int";
device_magnetometer_mz["unit"] = "mG";

JsonObject device_pressure = device.createNestedObject("pressure");
device_pressure["value"] = v16;
device_pressure["type"] = "float";
device_pressure["unit"] = "mb";

JsonObject device_altitude = device.createNestedObject("altitude");
device_altitude["value"] = v17;
device_altitude["type"] = "float";
device_altitude["unit"] = "m";

JsonObject device_temperature = device.createNestedObject("temperature");
device_temperature["value"] = v18;
device_temperature["type"] = "float";
device_temperature["unit"] = "°C";

//char info[1200];
String info;
serializeJson(doc, info);
//Serial.println(info);
return info;
 
  }//fin mensajes JSON

String tiempo(){
  while(!tiempoClient.update()){
      tiempoClient.forceUpdate();
    }
  
  //tomo la hora y la fecha
  formato = tiempoClient.getFormattedDate();

  //obtengo la fecha
  int splitTiempo = formato.indexOf("T");
  fecha = formato.substring(0, splitTiempo);

  //obtengo la hora
  hora = formato.substring(splitTiempo+1, formato.length()-1);
  String fecha_hora = fecha+" "+ hora ;
  return fecha_hora;
    
 }//fin tiempo

//----------------------------------------------------------------------envio de datos -----------------------------------------------------------------------------
void EnvioDatosCliente(String info){
  HTTPClient http;

  http.begin("http://192.168.56.1:80/");// http://11.11.27.131:8080/ IP defino destino para la solicitud http
  http.addHeader("Content-Type", "application/json");//especifico el orden de llegada de los datos

  //envio la solicitud POST real (si es 201 llego bien al servidor, si es < -1 hubo un error)
  int httpSolicitud = http.POST(info);

  if(httpSolicitud > 0){
      //respuesta solicitud
      String respuesta = http.getString();
      Serial.println();
      Serial.println();
    }

      else{
              Serial.print("Error al enviar POST: "); 
              Serial.println(httpSolicitud);
           }

           http.end();
}//fin EnvioDatosCliente
//----------------------------------------------------------------------fin envio de datos -----------------------------------------------------------------------------
