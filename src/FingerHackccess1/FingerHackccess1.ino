//      ___    __                          ______           _   _                 
//     /   |  / /   ______ __________     /_  __/___  _____(_) (_)___ _____  ____ 
//    / /| | / / | / / __ `/ ___/ __ \     / / / __ \/ ___/ / / / __ `/ __ \/ __ \
//   / ___ |/ /| |/ / /_/ / /  / /_/ /    / / / /_/ / /  / / / / /_/ / / / / /_/ /
//  /_/  |_/_/ |___/\__,_/_/   \____/    /_/  \____/_/  /_/_/ /\__,_/_/ /_/\____/ 
//                                                       /___/                    
//alvarotorijano@gmail.com

/* ____                                       __  __                    __                                             
/\  _`\   __                               /\ \/\ \                  /\ \                                            
\ \ \L\_\/\_\    ___      __      __   _ __\ \ \_\ \     __      ___ \ \ \/'\     ___    ___     __    ____    ____  
 \ \  _\/\/\ \ /' _ `\  /'_ `\  /'__`\/\`'__\ \  _  \  /'__`\   /'___\\ \ , <    /'___\ /'___\ /'__`\ /',__\  /',__\ 
  \ \ \/  \ \ \/\ \/\ \/\ \L\ \/\  __/\ \ \/ \ \ \ \ \/\ \L\.\_/\ \__/ \ \ \\`\ /\ \__//\ \__//\  __//\__, `\/\__, `\
   \ \_\   \ \_\ \_\ \_\ \____ \ \____\\ \_\  \ \_\ \_\ \__/.\_\ \____\ \ \_\ \_\ \____\ \____\ \____\/\____/\/\____/
    \/_/    \/_/\/_/\/_/\/___L\ \/____/ \/_/   \/_/\/_/\/__/\/_/\/____/  \/_/\/_/\/____/\/____/\/____/\/___/  \/___/ 
                          /\____/                                      
                          \_/__/                                       
*/

//DEPURACION
//#define DEBUG_TACTIL
#define DEBUG

//BOARD
#ifdef ESP_H
#define NODE_MCU
#endif

#define VERSION_FINGERHACKCCESS "V 0.7"
#define LIGHT_CONTROL false
#define UMBRAL_TACTIL 1000
#define DHT_THRESHOLD 500
#define TIEMPO_BRILLO         5.0      //Tiempo en segundos que se mantendra la pantalle encendida al mostrar un mensaje
#define TIEMPO_MENSAJE        3.5      //Tiempo que se mantendra en la pantalla el mensaje
#define WIFI_SSID   "ToryAP"
#define WIFI_PASS   "12345678900"
#define DDNS_TIMEOUT 1000
#define FINGERREADER_CHECK_TIMEOUT 5000
#define WIFI_CHECK_TIMEOUT 500
#define NTP_PORT 2390
#define NTP_SERVER "pool.ntp.org"
#define NTP_CHECK 100
#define NTP_SEND  2000

#ifdef NODE_MCU

#define DHTPIN D3
#define FINGERPRINT_TX D5
#define FINGERPRINT_RX D6
#define CAPACITIVE_SND D8
#define CAPACITIVE_RCV D7
#define PIN_PUERTA D4
#define SERIAL_BAUDRATE 74880
#define NTP_PACKET_SIZE 48 // NTP time stamp is in the first 48 bytes of the message

#else

#define DHTPIN 2
#define FINGERPRINT_TX 3
#define FINGERPRINT_RX 4
#define CAPACITIVE_SND A1
#define CAPACITIVE_RCV A0
#define PIN_PUERTA A2
#define SERIAL_BAUDRATE 9600

#endif

#define DS3231_I2C_ADDRESS  0x68
#define LCD16X2_I2C_ADDRESS 0x3F

#define DHTTYPE 22

#pragma  once

#ifdef NODE_MCU
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include "PietteTech_DHT.h"
#include <TimeLib.h>
#include <WiFiUdp.h>
#else
#include <DHT.h>
#endif

#include <Keypad.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <CapacitiveSensor.h>
#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>

//Prototipos
void mostrarMensaje();
void encenderLCD();
float getTemperature();
byte bcdToDec(byte);
byte decToBcd(byte);
void readDS3231time(byte *, byte *, byte *, byte *, byte *, byte *, byte *);
byte comparaOpcion(char * entrada, String * opciones, byte * longitud, byte cant, byte * p);
void setDS3231time(byte second_ds3231, byte minute_ds3231, byte hour_ds3231, byte dayOfWeek, byte dayOfMonth_ds3231, byte month_ds3231, byte year_ds3231);
void mostrarCadena(const char * entrada);
int leerHuella(void);
void hour_ds3231Option(byte i, String comando, byte * hour_ds3231, byte * minute_ds3231, byte * second_ds3231, byte * dayOfMonth_ds3231, byte * month_ds3231, byte * year_ds3231, byte * dayOfWeek);

#ifdef NODE_MCU
void showDHTError(int result);
void dht_wrapper();
#endif

//Text
 const PROGMEM char fech_o_invalid[]      = {"\nFormato de hora o fecha incorrecto"};
 const PROGMEM char reloj_actualizado[]   = {"\nSe ha actualizado el reloj con los siguientes datos:\n"};

//Estas son las definiciones del hardware
/****************************************/

//Mapa de caracteres para el teclado
#define NUM_ROW 4
#define NUM_COL 4

char keymap[4][4]= 
{{'A', 'B', 'C', 'D'}, 
 {'E', 'F', 'G', 'H'}, 
 {'I', 'J', 'K', 'L'},
 {'M', 'N', 'O', 'P'}};

//Pines que usa el teclado.
byte rowPins[NUM_ROW] = {9,8,7,6}; //Rows 0 to 3
byte colPins[NUM_COL]= {2,3,4,5}; //Columns 0 to 3

//Esta es la instancia del teclado
Keypad teclado= Keypad(makeKeymap(keymap), rowPins, colPins, NUM_ROW, NUM_COL);

//Este es el sensor de temperatura y humedad
#ifdef NODE_MCU
PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper);
WiFiServer server(80);
IPAddress timeServerIP; // NTP server address
WiFiUDP udp;
#else
DHT dht(DHTPIN, DHTTYPE);
#endif



//Sensor tactil
CapacitiveSensor tactil = CapacitiveSensor(CAPACITIVE_SND, CAPACITIVE_RCV);  

//Lector de huellas dactilares
SoftwareSerial mySerial(FINGERPRINT_TX, FINGERPRINT_RX);
Adafruit_Fingerprint lector = Adafruit_Fingerprint(&mySerial);

//Lcd 16X2
//                     Addr,                En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity
LiquidCrystal_I2C lcd( LCD16X2_I2C_ADDRESS,  2,  1,  0,  4,  5,  6,  7,           3, POSITIVE );

//Estas son las macros para la retroiluminacion del LCD
#define ON  1
#define OFF 0


//Variables globales
  byte second_ds3231, minute_ds3231, hour_ds3231, dayOfWeek, dayOfMonth_ds3231, month_ds3231, year_ds3231; //Fecha y hora actuales
  byte sec_anterior = 0;
  bool flag_luz = LIGHT_CONTROL;
  unsigned long iluminacion;
  unsigned long mensaje;
  unsigned long millis_actuales;
  unsigned long ultima_dht;

  #ifdef NODE_MCU
  byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
  #endif

//ICONOS//
byte termometro[8] = 
{
    B00100,  //Este es el icono del termometro
    B01010,
    B01010,
    B01110,
    B01110,
    B11111,
    B11111,
    B01110,
    };

    byte grados[8] = 
{
    B00110, // este es el icono de º  //
    B01001,                           //
    B01001,                           //
    B00110,
    B00000,
    B00000,
    B00000,
    B00000,
};


byte gota[8] = //icono de la gota
{
    B00100,
    B00100,
    B01010,
    B01010,
    B10001,
    B10001,
    B10001,
    B01110,
};
    

void setup() {
  
  iluminacion = mensaje = millis_actuales = ultima_dht = millis();
  
  pinMode(PIN_PUERTA, OUTPUT);
  Serial.begin(SERIAL_BAUDRATE);
  
  #ifdef NODE_MCU
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  udp.begin(NTP_PORT);
  #else
  dht.begin();
  #endif
  
  lcd.begin(16,2);
  lcd.backlight();
  lcd.createChar(1,grados);
  lcd.createChar(2,gota);
  lcd.createChar(3,termometro);
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x07); // move pointer to SQW address
  Wire.write(0x10); // sends 0x10 (hex) 00010000 (binary) to control register - turns on square wave
  Wire.endTransmission();
  
  tactil.set_CS_AutocaL_Millis(0x400);

  lector.begin(57600);

  

  lcd.clear();
  lcd.print("FingerHackccess");
  lcd.setCursor(5,1);
  lcd.print(VERSION_FINGERHACKCCESS);
  delay(2000);
    //Este Bloque me permite ver la salida del sensor tactil
  #ifdef DEBUG_TACTIL
  while(1){
    long start = millis();
    long total1 =  tactil.capacitiveSensor(30);
    Serial.print(millis() - start);        // check on performance in millisecond_ds3231s
    Serial.print("\t");                    // tab character for debug windown spacing
    Serial.println(total1);                  // print sensor output 1
    delay(10);
  }
  #endif
}

void loop() {
  char comando [40] = {'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};
  String pruebas [1] = "HORA";
  byte longitudes [1] = {4};
  byte i = 0, hour_ds3231, minute_ds3231, second_ds3231, dayOfMonth_ds3231, month_ds3231, year_ds3231, dayOfWeek;
  static float temp, dht_t, dht_h;
  
  #ifdef NODE_MCU
  int result;
  int static cb;
  static bool is_wifi_connected = false, is_fingerReader_Connected = false, is_NTP_updated = false;
  static unsigned long millis_wifi_check = millis(), millis_fingerReader_check = millis(), millis_NTP_check = millis(), millis_NTP_send = millis();
  HTTPClient http;
  #endif
  millis_actuales = millis();
  #ifdef NODE_MCU

  if( (is_wifi_connected == false) && ((millis_actuales - millis_wifi_check)> WIFI_CHECK_TIMEOUT)){
    if (WiFi.status() == WL_CONNECTED){
      
      lcd.clear();
      lcd.setBacklight(ON);
      lcd.print(WIFI_SSID);
      lcd.setCursor(0,1);
      lcd.print(WiFi.localIP());
      udp.begin(NTP_PORT);
      is_wifi_connected = !is_wifi_connected; //This is a flag
      server.begin();
      
      delay(1000);
    }
      else{
        
      }
      millis_wifi_check = millis_actuales;
  }

  if((is_NTP_updated == false) && (is_wifi_connected == true) && ((millis_actuales - millis_NTP_send) > NTP_SEND)){
    WiFi.hostByName(NTP_SERVER, timeServerIP); 
    sendNTPpacket(timeServerIP); // send an NTP packet to a time server
    #ifdef DEBUG
    Serial.println("Pidiendo NTP");
    #endif
    millis_NTP_send = millis_NTP_check = millis_actuales;
  }

  if((is_NTP_updated == false) && (is_wifi_connected == true) && ((millis_actuales - millis_NTP_check) > NTP_CHECK)){
    cb = udp.parsePacket();
  if (!cb) {
    #ifdef DEBUG
    Serial.println("no NTP packet yet");
    #endif
    millis_NTP_check = millis_actuales + NTP_CHECK;
  }
  else {
    is_NTP_updated = true;
    #ifdef DEBUG
    Serial.print("packet received, length=");
    Serial.println(cb);
    #endif
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    #ifdef DEBUG
    Serial.println(epoch);
    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if ( ((epoch % 3600) / 60) < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ( (epoch % 60) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
    #endif
    convertUnixTime (epoch, & hour_ds3231, & minute_ds3231, & second_ds3231, & dayOfMonth_ds3231, & month_ds3231,& year_ds3231, & dayOfWeek);
    #ifdef DEBUG
    Serial.println("Fecha: ");
    Serial.print(dayOfMonth_ds3231);
    Serial.print("/");
    Serial.print(month_ds3231);
    Serial.print("/");
    Serial.print(year_ds3231);
    Serial.print("  ");
    Serial.print(hour_ds3231);
    Serial.print(":");
    Serial.print(minute_ds3231);
    Serial.print(":");
    Serial.print(second_ds3231);
    #endif
    setDS3231time(second_ds3231, minute_ds3231, hour_ds3231, dayOfWeek, dayOfMonth_ds3231, month_ds3231, year_ds3231);
    lcd.clear();
    lcd.print("NTP hour UPDATED");
    encenderLCD();
    mostrarMensaje();
  }
  }
  
  #endif
  if((is_fingerReader_Connected == false) && ( (millis_actuales - millis_fingerReader_check)> FINGERREADER_CHECK_TIMEOUT)){
    if (lector.verifyPassword()){
      #ifdef DEBUG
      Serial.println("Found fingerprint reader");
      #endif
      lcd.clear();
      lcd.print("Lector ok");
      mostrarMensaje();
      encenderLCD();
      is_fingerReader_Connected = !is_fingerReader_Connected;
      delay (1000);
      }
    else{
      #ifdef DEBUG
      Serial.println("Fingerprint reader not found");
      #endif
      lcd.clear();
      lcd.setCursor(3,0);
      lcd.print("Lector no");
      lcd.setCursor(3,1);
      lcd.print("encontrado");
      millis_fingerReader_check = millis_actuales;
    }
  }
  
  
  if((millis_actuales - ultima_dht) > DHT_THRESHOLD){
    
  #ifdef NODE_MCU
  
  result = DHT.acquireAndWait(0);

  showDHTError(result);
  
  dht_h = DHT.getHumidity();
  dht_t = DHT.getCelsius();
  
  #else
  
  dht_h = dht.readHumidity();
  dht_t = dht.readTemperature();
  
  #endif
  ultima_dht = millis_actuales;
  }

  if(Serial.available()){
      delay (400);
      
       while (Serial.available()){
          comando[i] = Serial.read();
          Serial.print(comando[i]);
          i++;
       }
       Serial.flush();

  switch (comparaOpcion(comando, pruebas, longitudes, (byte)sizeof(longitudes),&i)){
    case 1:
    hour_ds3231Option(i, comando, &hour_ds3231, &minute_ds3231, &second_ds3231, &dayOfMonth_ds3231, &month_ds3231, &year_ds3231, &dayOfWeek);
    break;
  }
}
  if ((millis_actuales > iluminacion) && (flag_luz == true)){
      lcd.setBacklight(OFF);
    }

  readDS3231time( &second_ds3231, &minute_ds3231, &hour_ds3231, &dayOfWeek, &dayOfMonth_ds3231, &month_ds3231, &year_ds3231); // Lo primero es leer la fecha y la hora del reloj

    //mostramos los datos en la pantalla si procede
    if((second_ds3231 != sec_anterior) &&(millis_actuales > mensaje)){
      sec_anterior = second_ds3231;
      temp = getTemperature();
      
      lcd.clear();
      lcd.print(hour_ds3231);
      lcd.print(":");
      lcd.print(minute_ds3231);
      lcd.print(":");
      lcd.print(second_ds3231);
      lcd.print(" ");
      lcd.print(temp);
      lcd.write(1);
      lcd.print("C");
      lcd.setCursor(0,1);
      lcd.print(dht_t);
      lcd.write(3);
      lcd.print("  ");
      lcd.print(dht_h);
      lcd.write(2);
           
    }

  if(is_fingerReader_Connected && tactil.capacitiveSensor(30) > UMBRAL_TACTIL){
    if(leerHuella() == 1){
      lcd.clear();
      lcd.print("Acceso Permitido");
      lcd.setCursor(0,1);
      lcd.print(lector.fingerID);
      digitalWrite(PIN_PUERTA, HIGH);
      delay(5000);
      digitalWrite(PIN_PUERTA, LOW);
      lcd.clear();
      lcd.print("Esperando dedo");
    }
    else if(leerHuella() == -1){
      lcd.clear();
      lcd.print("Acceso Denegado");
      delay(5000);
    }
  }
}

//Funciones

//FUNCION QUE RESETEA LA PLACA//
//----------------------------//
void resetearLector(void){
  #ifdef NODE_MCU
    int i;
    i=0;
  #else
    asm volatile ("jmp 0");  
  #endif
  
}

//FUNCION QUE LEE UNA HUELLA Y LA COMPARA//
//---------------------------------------//


int leerHuella(void){
  //La lectura de la huella se subdivide en tres fases:
  //1-Obtemcion de la imagen
  //2-Compresion y procesado de la imagen
  //3-Comparacion de los resultados con las huellas almacenadas

  byte p;

  switch (lector.getImage()){
    case FINGERPRINT_OK:
         //imagen obtenida
        switch (lector.image2Tz()){
          case FINGERPRINT_OK:
            //Serial.println("Image converted");
            //imagen convertida

            p = lector.fingerFastSearch();
            if (p == FINGERPRINT_OK) {
              //Esto es que la huella esta permitida
              return 1;
            } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
              lcd.clear();
              lcd.setCursor(4,0);
              lcd.print("Error de");
              lcd.setCursor(2,1);
              lcd.print("comunicacion");
              mostrarMensaje();
              encenderLCD();
              return 0;
            } else if (p == FINGERPRINT_NOTFOUND) {
              return -1;
            } else {
              lcd.clear();
              lcd.print("Error desconocido");
              mostrarMensaje();
              encenderLCD();
              return 0;
            }
            
            break;
            
          case FINGERPRINT_IMAGEMESS:
            lcd.clear();
            lcd.print("Imagen borrosa");
            mostrarMensaje();
            encenderLCD();
            break;
            
          case FINGERPRINT_PACKETRECIEVEERR:
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("Error de");
            lcd.setCursor(2,1);
            lcd.print("comunicacion");
            mostrarMensaje();
            encenderLCD();
            break;
            
          default:
            lcd.clear();
            lcd.print("Error desconocido");
            mostrarMensaje();
            encenderLCD();
            break;
        }
      //Serial.println("Image taken");
      break;
      
    case FINGERPRINT_NOFINGER:
      lcd.clear();
      lcd.setCursor(4,0);
      lcd.print("Dedo no");
      lcd.setCursor(3,1);
      lcd.print("detectado");
      encenderLCD();
      mostrarMensaje();
      break;
      
    case FINGERPRINT_PACKETRECIEVEERR:
      lcd.clear();
      lcd.setCursor(4,0);
      lcd.print("Error de");
      lcd.setCursor(2,1);
      lcd.print("comunicacion");
      mostrarMensaje();
      encenderLCD();
      break;
      
    case FINGERPRINT_IMAGEFAIL:
      lcd.clear();
      lcd.setCursor(4,0);
      lcd.print("Error de");
      lcd.setCursor(5,1);
      lcd.print("imagen");
      mostrarMensaje();
      encenderLCD();
      break;
      
    default:
      lcd.clear();
      lcd.print("Error desconocido");
      mostrarMensaje();
      encenderLCD();
      break;
    
  }//Aqui se cierra el primer switch
  return 0;
}//Termina la función


//Funcion que mantiene el mensaje en pantalla//
//-------------------------------------------//
void mostrarMensaje(void){
  mensaje = millis() + (TIEMPO_MENSAJE * 1000);
}

//FUNCION QUE ENCIENDE LA PANTALLA DETERMINADO TIEMPO//
//---------------------------------------------------//
void encenderLCD(void){ 
  lcd.setBacklight(ON);
  iluminacion = millis() + (TIEMPO_BRILLO * 1000);
}

//Esta funcion permite usar el teclado matricial como un T9//
//---------------------------------------------------------//
char caracter (char boton, int i){
  boton = (int)boton - 65;
  i = i%5;
  const char caracteres [5][16] = {'1', '2','3','A','4','5','6','B','7','8','9','C','*','0','#','D',
                                   '.', 'a','d','A','g','j','m','B','p','t','w','C','*',' ','#','D',
                                   ',', 'b','e','A','h','k','n','B','q','u','x','C','*','+','#','D',
                                   '-', 'c','f','A','i','l','ñ','B','r','v','y','C','*','_','#','D',
                                   '\'',':',';','A','{','}','o','B','s','ç','z','C','*','/','#','D'};

  return caracteres[i][boton]; 
                         
}


//FUNCION QUE LEE LA HORA//
//-----------------------//

void readDS3231time(byte *second_ds3231, byte *minute_ds3231, byte *hour_ds3231, byte *dayOfWeek, byte *dayOfMonth_ds3231, byte *month_ds3231, byte *year_ds3231)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second_ds3231 = bcdToDec(Wire.read() & 0x7f);
  *minute_ds3231 = bcdToDec(Wire.read());
  *hour_ds3231 = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth_ds3231 = bcdToDec(Wire.read());
  *month_ds3231 = bcdToDec(Wire.read());
  *year_ds3231 = bcdToDec(Wire.read());
}

//FUNCION QUE OBTENE LA TEMPERATURA//
//----------------------------------//
float getTemperature()
{
    byte temperature;

    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(uint8_t(0x11));
    Wire.endTransmission();
    Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
    temperature = Wire.read();

    return float(temperature) + 0.25*(Wire.read()>>6);
}

//FUNCION QUE CONVIERTE NUMEROS DECIMALES A BINARIOS//
//--------------------------------------------------//

byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}


//FUNCION QUE CONVIERTE NUMEROS BINARIOS EN DECIMALES//
//---------------------------------------------------//

byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

//FUNCION QUE DEVUELVE LA OPCION//
//------------------------------//
byte comparaOpcion(char * entrada, String * opciones, byte * longitud, byte cant, byte * p){
          byte i=0, j, k, c;
          while((entrada [i] == ' ')||(entrada [i] == '\n'))
          i++;

        *p = i;
        for(k=0; k<cant; k++){
          c=0;
          for(j=i; j<i+longitud[k] ; j++){
            if(toupper(entrada[j]) == opciones[k][j-i])
            c++;
            if( c==longitud[k] )
            return (k+1);
            }
        }
        return 0;
}

//Funcion que muestra por la interfaz serie una cadena guardada en la memoria de programa//
//---------------------------------------------------------------------------------------//
void mostrarCadena(const char * entrada){
    // read back a char
    char myChar;
    int k;
  int len = strlen_P(entrada);
  for (k = 0; k < len; k++)
  {
    myChar =  pgm_read_byte_near(entrada + k);
    Serial.print(myChar);
  }
}

//FUNCION QUE ESTABLECE LA HORA//
//-----------------------------//
void setDS3231time(byte second_ds3231, byte minute_ds3231, byte hour_ds3231, byte dayOfWeek, byte dayOfMonth_ds3231, byte month_ds3231, byte year_ds3231)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the second_ds3231s register
  Wire.write(decToBcd(second_ds3231)); // set second_ds3231s
  Wire.write(decToBcd(minute_ds3231)); // set minute_ds3231s
  Wire.write(decToBcd(hour_ds3231)); // set hour_ds3231s
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth_ds3231)); // set date (1 to 31)
  Wire.write(decToBcd(month_ds3231)); // set month_ds3231
  Wire.write(decToBcd(year_ds3231)); // set year_ds3231 (0 to 99)
  Wire.endTransmission();
}

// This wrapper is in charge of calling
// must be defined like this for the lib work
#ifdef NODE_MCU
  void dht_wrapper() {
    DHT.isrCallback();
  }
  #endif

  void showDHTError(int result){
    switch (result) {
        case DHTLIB_OK:
            /*#ifdef DEBUG
            Serial.println("DHT: OK");
            #endif*/
            break;
        case DHTLIB_ERROR_CHECKSUM:
            #ifdef DEBUG
            Serial.println("DHT: Error\n\r\tChecksum error");
            #endif
            break;
        case DHTLIB_ERROR_ISR_TIMEOUT:
            #ifdef DEBUG
            Serial.println("DHT: Error\n\r\tISR time out error");
            #endif
            break;
        case DHTLIB_ERROR_RESPONSE_TIMEOUT:
            #ifdef DEBUG
            Serial.println("DHT: Error\n\r\tResponse time out error");
            #endif
            break;
        case DHTLIB_ERROR_DATA_TIMEOUT:
            #ifdef DEBUG
            Serial.println("DHT: Error\n\r\tData time out error");
            #endif
            break;
        case DHTLIB_ERROR_ACQUIRING:
            #ifdef DEBUG
            Serial.println("DHT: Error\n\r\tAcquiring");
            #endif
            break;
        case DHTLIB_ERROR_DELTA:
            #ifdef DEBUG
            Serial.println("DHT: Error\n\r\tDelta time to small");
            #endif
            break;
        case DHTLIB_ERROR_NOTSTARTED:
            #ifdef DEBUG
            Serial.println("DHT: Error\n\r\tNot started");
            #endif
            break;
        default:
            #ifdef DEBUG
            Serial.println("DHT: Unknown error");
            #endif
            break;
    }
  }

void hour_ds3231Option(byte i, String comando, byte * hour_ds3231, byte * minute_ds3231, byte * second_ds3231, byte * dayOfMonth_ds3231, byte * month_ds3231, byte * year_ds3231, byte * dayOfWeek){

    *hour_ds3231       = ((((byte)(comando[i + 5])) -48) * 10) + (((byte)(comando[i + 6]) -48));
    *minute_ds3231     = ((((byte)(comando[i + 8])) -48) * 10) + (((byte)(comando[i + 9]) -48));
    *second_ds3231     = ((((byte)(comando[i+ 11])) -48) * 10) + (((byte)(comando[i+ 12]) -48));
    *dayOfMonth_ds3231 = ((((byte)(comando[i+ 14])) -48) * 10) + (((byte)(comando[i+ 15]) -48));
    *month_ds3231      = ((((byte)(comando[i+ 17])) -48) * 10) + (((byte)(comando[i+ 18]) -48));
    *year_ds3231       = ((((byte)(comando[i+ 22])) -48) * 10) + (((byte)(comando[i+ 23]) -48));
    *dayOfWeek  = ((((byte)(comando[i+ 25])) -48));

    //Se comprueba que los valores esten en los rangos y que los valores tomados provengan de caracteres numericos
    if ((*hour_ds3231 >24) || (*minute_ds3231 >60) || (*second_ds3231 >60) || (*dayOfMonth_ds3231 > 31) || (*month_ds3231 >12) || (*year_ds3231 >50) || (*dayOfWeek >7) || !((comando[i +  5] > 47) && (comando[i +  5] <58)) || !((comando[i +  6] > 47) && (comando[i +  6] <58)) || !((comando[i +  8] > 47) && (comando[i +  8] <58)) || !((comando[i +  9] > 47) && (comando[i +  9] <58)) || !((comando[i + 11] > 47) && (comando[i + 11] <58)) || !((comando[i + 12] > 47) && (comando[i + 12] <58)) || !((comando[i + 14] > 47) && (comando[i + 14] <58)) || !((comando[i + 15] > 47) && (comando[i + 15] <58)) || !((comando[i + 17] > 47) && (comando[i + 17] <58)) || !((comando[i + 18] > 47) && (comando[i + 18] <58)) || !((comando[i + 20] > 47) && (comando[i + 20] <58)) || !((comando[i + 21] > 47) && (comando[i + 21] <58)) || !((comando[i + 23] > 47) && (comando[i + 23] <58))){
      mostrarCadena(fech_o_invalid);
      lcd.clear();
      lcd.print("  Hora o fecha");
      lcd.setCursor(4,1);
      lcd.print("INVALIDA");
      mostrarMensaje();
      encenderLCD();
      }
    else{
      setDS3231time(*second_ds3231, *minute_ds3231, *hour_ds3231, *dayOfWeek, *dayOfMonth_ds3231, *month_ds3231, *year_ds3231);
      readDS3231time(second_ds3231, minute_ds3231, hour_ds3231, dayOfWeek, dayOfMonth_ds3231, month_ds3231, year_ds3231);
      mostrarCadena(reloj_actualizado);

       Serial.print(*hour_ds3231);
       Serial.print(":"); 
       Serial.print(*minute_ds3231);
       Serial.print(":");
       Serial.print(*second_ds3231);
       Serial.print(" ");
       Serial.print(*dayOfMonth_ds3231);
       Serial.print("/");
       Serial.print(*month_ds3231);
       Serial.print("/20");
       Serial.print(*year_ds3231);

       lcd.clear();
       encenderLCD();
       mostrarMensaje();                    
       lcd.print("Hora actualiaza");
       lcd.setCursor(4,1);
       lcd.print(*hour_ds3231);
       lcd.print(":");
       lcd.print(*minute_ds3231);
       lcd.print(":");
       lcd.print(*second_ds3231);
       delay(1000);
       readDS3231time(second_ds3231, minute_ds3231, hour_ds3231, dayOfWeek, dayOfMonth_ds3231, month_ds3231, year_ds3231);
       lcd.setCursor(11,1);
       lcd.print(*second_ds3231);
       delay(1000);
       lcd.clear();
       lcd.print(" Fecha cambiada");
       lcd.setCursor(3,1);
       lcd.print(*dayOfMonth_ds3231);
       lcd.print("/");
       lcd.print(*month_ds3231);
       lcd.print("/20");
       lcd.print(*year_ds3231);
       mensaje = millis() + 2000;
    }  
}

#ifdef NODE_MCU
void sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void convertUnixTime (time_t t, byte * hour_ds3231, byte * minute_ds3231, byte * second_ds3231, byte * dayOfMonth_ds3231, byte * month_ds3231, byte * year_ds3231, byte * dayOfWeek){
  * dayOfMonth_ds3231 = day(t);
  * month_ds3231 = month(t);
  * year_ds3231 = year(t);
  * hour_ds3231 = hour(t);
  * minute_ds3231 = minute(t);
  * second_ds3231 = second(t);
  * dayOfWeek = weekday(t);
  * month_ds3231++;
}
#endif

