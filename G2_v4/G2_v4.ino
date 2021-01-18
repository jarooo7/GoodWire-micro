#include <SoftwareSerial.h>
#include <SimpleDHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

//biblioteki do GSM
#include <Geo.h>
#include <Http.h>
#include <Parser.h>
#include <Sim800.h>
#include <ArduinoJson.h>

SoftwareSerial pmsSerial(3, 2);
SoftwareSerial gpsSerial(6, 5);

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;
uint16_t  pm10;
uint16_t  pm25;
uint16_t  pm100;
unsigned long time_r = 0;
char location[25] = "-----------------------\0";
short rm;
short rt;

  float temperature = 0;
  float temperature2 = 0;
  float pressure = 0;
  float humidity = 0;

byte index[2] = {0, 0};
 char body[145];

bool flagG = false;
bool flagA = false;
    

unsigned long lastRunTime = 0; 
unsigned long waitForRunTime = 0;

SimpleDHT22 dht22(9);
Adafruit_BMP085 bmp;
HTTP http(9600, 8, 7, 10);

void setup() {
   pinMode(11,OUTPUT);
    digitalWrite(11, HIGH);
  Serial.begin(9600);
  pmsSerial.begin(9600);
  gpsSerial.begin(9600);
  bmp.begin();
  pinMode(4,OUTPUT);
 


}

void loop() {
  digitalWrite(4, LOW);
  digitalWrite(11, LOW);
  if ((millis() - time_r> (1000L*60*3)) ||Serial.read()=='A') {
    digitalWrite(11, HIGH);
    delay(2000);
    gpsSerial.listen();
    delay(200);
    strcpy(location,"-----------------------\0");
    gps();
    digitalWrite(11, LOW);
    Serial.println(location);
    delay(200);
    rm = analogRead(A1); 
    //VRL_MQ135 = analogRead(A1)*(5.0/1023.0); 
    //Rs_MQ135 = ((5.0/VRL_MQ135)-1)*(10); 
    //ratio_MQ135 = Rs_MQ135/Ro_MQ135;
    //float CO = 112.89808 * pow(ratio_MQ135,  -2.868463517);
    //float NO =  34.69756084 * pow(ratio_MQ135, -3.422829698);
    Serial.println(analogRead(A0));
    rt = analogRead(A0); 

    time_r = millis();
    temperature2 = bmp.readTemperature();
    pressure = bmp.readPressure();
    readDHT(&temperature, &humidity);
    Serial.print(temperature); Serial.print(" *C, ");
    Serial.print(humidity); Serial.println(" RH%");
    Serial.print(temperature2); Serial.print(" *C, ");
    Serial.print(pressure / 100); Serial.println(" hPa");

    pmsSerial.listen();
    delay(200);
    digitalWrite(4, HIGH);
    delay(3000);
    s_pm();
    Serial.print("PM10: "); Serial.print(pm100); Serial.println(" um/m3");
    Serial.print("PM2.5: "); Serial.print(pm25); Serial.println(" um/m3");
    Serial.print("PM1: "); Serial.print(pm10); Serial.println(" um/m3");
    digitalWrite(4, LOW);
    sprintf(body, "{\"d\":\"LV9H-U4YV-ZV0S-LYWQ-2342\",\"p1\":%d,\"p25\":%d,\"p10\":%d,\"l\":\"%s\",\"t\":%d.%d,\"p\":%d.%d,\"h\":%d.%d,\"rt\":%d,\"rm\":%d}",data.pm10_env,data.pm25_env ,data.pm100_env, location,(int)((temperature+temperature2)/2), int(((float)((temperature+temperature2)/2)-(int)((temperature+temperature2)/2))*100),(int)(pressure / 100),int(((float)(pressure / 100)-(int)(pressure / 100))*10), (int)(humidity),int(((float)(humidity)-(int)(humidity))*10),rt,rm);
    Serial.println(body);
    Serial.println(sizeof(body));
   

   if (shouldTrackTimeEntry()) sendToServer(body);
  }

}

void s_pm(){
  uint16_t t_pm10[20];
  uint16_t t_pm25[20];
  uint16_t t_pm100[20];
  pm10=0;
  pm25=0;
  pm100=0;
  for(byte i=0; i<20;i++){
    while(!readPMSdata(&pmsSerial));
      t_pm10[i]=data.pm10_standard;
      t_pm25[i]=data.pm25_standard;
      t_pm100[i]=data.pm100_standard;
    }
    for(byte i=0; i<20;i++){
       pm10=pm10+t_pm10[i];
       pm25=pm25+t_pm25[i];
      pm100=pm100+t_pm100[i];
      }
       pm10=pm10/20;
       pm25=pm25/20;
      pm100=pm100/20;
  }


void gps() {
  while (location[0] == '-' || location[23] == '-' || location[25] == '0') {
    for (int i = 0; i < 3000; i++) {
      while (gpsSerial.available() > 0) {
        char gpsData = gpsSerial.read();
    Serial.print(gpsData);
        if (flagG && gpsData == 'A') {
          flagA = true;
        }
        else {
          flagG = false;
        }
        flagG = (gpsData == 'G');

        if (flagA) {
          if (index[1] >= 2 && gpsData != ',') {
            location[index[0]] = gpsData;
            index[0]++;
          }
          if (gpsData == ',') {
            index[1]++;
          }
          if (index[1] == 6) {
            flagG = false;
            flagA = false;
            index[1] = 0;
            index[0] = 0;
          }
        }
      }
    }
  }
}



void readDHT(float* temperature, float* humidity) {
  while (*temperature == (float)(0) || *humidity == (float)(0)) {
    dht22.read2(temperature, humidity, NULL);
  }
}

boolean readPMSdata(Stream *s) {
  
  if (! s->available()) {
    return false;
  }
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
  if (s->available() < 32) {
    return false;
  }
  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }
  memcpy((void *)&data, (void *)buffer_u16, 30);
  if (sum != data.checksum) {
    return false;
  }
  return true;
}


void print(const __FlashStringHelper *message, int code = -1){
  if (code != -1){
    Serial.print(message);
    Serial.println(code);
  }
  else {
    Serial.println(message);
  }
}

bool shouldTrackTimeEntry(){
  unsigned long elapsedTime = millis() - lastRunTime;
  print(F("czas: "), elapsedTime);
  return elapsedTime >= waitForRunTime;
}

void sendToServer(char* body){
  
  char response[32];

  Result result;

  print(F("Cofigure bearer: "), http.configureBearer("Internet"));
  result = http.connect();
  print(F("HTTP łaczenie: "), result);
  Serial.println("#######################33");
 Serial.println(body);
  do{
  result = http.post("goodwire.cba.pl/api/surveyd", body, response);
  print(F("HTTP POST: "), result);
    delay(2000);
    }while(result != SUCCESS);
  if (result == SUCCESS) {
    Serial.println(response);
    StaticJsonBuffer<32> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(response);
    lastRunTime = millis();
    waitForRunTime = root["waitForRunTime"];
  }
  delay(500);
  print(F("HTTP rozłaczone: "), http.disconnect());
}
