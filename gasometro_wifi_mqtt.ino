#include <WiFi.h>
#include "AsyncMqttClient.h"
#include <Wire.h>
#include "time.h"
//#include "Arduino.h"
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"
#include <Preferences.h>
#include <ESP32Time.h>
#include <Adafruit_BMP085.h>
//#include "pitches.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AsyncTCP.h>



//**********************************************************//
//* DECLARO DE FUN                                        **//
//**********************************************************//
void pedir_lahora(void);
void setup_rtc_ntp(void);


struct tm timeinfo;
ESP32Time rtc;


/// time
long unsigned int timestamp; // hora
const char *ntpServer = "south-america.pool.ntp.org";
const long gmtOffset_sec = -10800;
const int daylightOffset_sec = 0;

Preferences preferences;
DNSServer dnsServer;
AsyncWebServer server(80);

String ssid;
String password;
String Token_tel;
String Id_tel;

char ap_ssid[30] = "abc_mirko";
char ap_password[30] = "mirko1793";

char ap_token[50];

char ap_Id_tel[20];

bool is_setup_done = false;
bool valid_ssid_received = false;
bool valid_password_received = false;
bool wifi_timeout = false;

void connectToMqtt();

//////wifi
const char* ssidMQTT = "ORT-IoT";
const char* passwordMQTT = "OrtIOTnew22$";

const char name_device = 23;  ////device numero de grupo 5A 1x siendo x el numero de grupo
///                        5B 2x siendo x el numero de grupo

// Timers auxiliar variables//////////////////////////
unsigned long now = millis(); ///valor actual
unsigned long lastMeasure1 = 0; ///variable para contar el tiempo actual
unsigned long lastMeasure2 = 0; ///variable para contar el tiempo actual

const unsigned long interval_envio = 30000;//Intervalo de envio de datos mqtt
const unsigned long interval_leeo =  60000;//Intervalo de lectura de datos y guardado en la cola
int i = 0;



///variables ingresar a la cola struct
int indice_entra = 0;
int indice_saca = 0;
bool flag_vacio = 1;

/////mqqtt
#define MQTT_HOST IPAddress(10, 162, 24, 31)
#define MQTT_PORT 1883
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "mirko15"
char mqtt_payload[150] ;  /////
// Test MQTT Topic
#define MQTT_PUB "/esp32/datos_sensores"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

typedef struct
{
  long time;
  float T1;///tempe
  float G5;///gas 5
  float G7;// gas 7
  float Presion; //presion
  float luz;
  bool Alarma;
  float ruido;
} estructura ;
/////////////////
const int valor_max_struct = 1000; ///valor vector de struct
estructura datos_struct [valor_max_struct];///Guardo valores hasta que lo pueda enviar
estructura aux2 ;

void alarma();
void toneBuzzer(int pin, int frequency, int duration);
void displayoled();
void displayText(String text);

int getTemperaturaBmp();
int getPresionBmp();
int getMonoxidoCarbonoMq7();
int getGasMq5();
int getLuzTemt();

#define PIN_MQ7 34
#define PIN_MQ5 35

//---------buzzer-----------//
int freq = 3000;
int channel = 0;
int resolution = 8;

#define PIN_BUZZER 12
#define GAS_ALARMANTE 20
#define FREQUENCY 4000
#define BUZZER_OFF 0
#define BUZZER_ON 1
#define SOUND_ON            (1<<(resolution-1)) // 50% duty cycle
#define SOUND_OFF           0                         // 0% duty cycle

int estadoBuzzer = BUZZER_OFF;

//#define PIN_PWM_FRECUENCIA_BUZZER A2 //potenciometro de frecuencia
#define PIN_TEMT6000 33

//maquina de estados display

#define DISPLAY_TEMPERATURA 0
#define DISPLAY_PRESION 1
#define DISPLAY_GAS 2
#define DISPLAY_LUZ 3
#define DISPLAY_CARBONO 4
int estadoDisplay = DISPLAY_TEMPERATURA;


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_MOSI   23 //d1
#define OLED_CLK    18 //d0
#define OLED_DC     16
#define OLED_CS     5
#define OLED_RESET  17

Adafruit_BMP085 bmp;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

//--- TIMERS ---//
//1000000 = 1 segundo
#define TIMER_DISPLAY 1000000
hw_timer_t * timerDisplay = NULL;
bool timerTerminado = false;

void IRAM_ATTR onTimerDisplay() {
  // codigo:
  timerTerminado = true;
  Serial.println("timer terminado");
}

/////*********************************************************************/////
////////////////////////////setup wifi/////////////////////////////////////////
/////*********************************************************************/////
void setupmqtt()
{
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
}
////////////////////////////Envio de datos mqtt//////////////////////////////////////////
////////Funcion que envia valores cuando la estructura no este vacia ///////////////////
///////////////////////////////////////////////////////////////////////////////////////
void fun_envio_mqtt ()
{
  fun_saca ();////veo si hay valores nuevos
  if (flag_vacio == 0) ////si hay los envio
  {
    Serial.print("enviando");
    ////genero el string a enviar
    snprintf (mqtt_payload, 150, "%u&%ld&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%u", name_device, aux2.time, aux2.T1, aux2.G5, aux2.G7, aux2.Presion, aux2.luz, aux2.ruido, aux2.Alarma); //random(10,50)
    aux2.time = 0; ///limpio valores
    aux2.T1 = 0;
    aux2.G5 = 0;
    aux2.G7 = 0;
    aux2.Presion = 0;
    aux2.luz = 0;
    aux2.Alarma = 0;
    aux2.ruido = 0;

    Serial.print("Publish message: ");
    Serial.println(mqtt_payload);
    // Publishes Temperature and Humidity values
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB, 1, true, mqtt_payload);
  }
  else
  {
    Serial.println("no hay valores nuevos");
  }
}///////////////////////////////////////////////////

///////////////////////////////////////////////////
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssidMQTT, passwordMQTT);
}///////////////////////////////////////////////////
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}///////////////////////////////////////////////////

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}///////////////////////////////////////////////////
////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}///////////////////////////////////////////////////

////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}
////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}///////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
/////////////Funcion que saca un valor de la estructura para enviar //////
///////////////////////////////////////////////////////////////////////
void fun_saca () {
  if (indice_saca != indice_entra)
  {
    aux2.time = datos_struct[indice_saca].time;
    aux2.T1 = datos_struct[indice_saca].T1;
    aux2.G5 = datos_struct[indice_saca].G5;
    aux2.G7 = datos_struct[indice_saca].G7;
    aux2.Presion = datos_struct[indice_saca].Presion;
    aux2.luz = datos_struct[indice_saca].luz;
    aux2.Alarma = datos_struct[indice_saca].Alarma;
    aux2.ruido = datos_struct[indice_saca].ruido;

    flag_vacio = 0;

    Serial.println(indice_saca);
    if (indice_saca >= (valor_max_struct - 1))
    {
      indice_saca = 0;
    }
    else
    {
      indice_saca++;
    }
    Serial.print("saco valores de la struct isaca:");
    Serial.println(indice_saca);
  }
  else
  {
    flag_vacio = 1; ///// no hay datos
  }
  return ;
}
/////////////////////////////////////////////////////////////////////
/////////////funcion que ingresa valores a la cola struct///////////
///////////////////////////////////////////////////////////////////
void fun_entra (void)
{
  if (indice_entra >= valor_max_struct)
  {
    indice_entra = 0; ///si llego al maximo de la cola se vuelve a cero
  }
  //////////// timestamp/////// consigo la hora
  Serial.print("> NTP Time:");
  timestamp =  time(NULL);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;  //// si no puede conseguir la hora
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  ///////////////////////// fin de consigo la hora
  datos_struct[indice_entra].time = timestamp;
  datos_struct[indice_entra].T1 = getTemperaturaBmp(); /// leeo los datos
  datos_struct[indice_entra].G5 = getGasMq5(); //// se puede pasar por un parametro
  datos_struct[indice_entra].G7 = getMonoxidoCarbonoMq7();
  datos_struct[indice_entra].luz = getLuzTemt();
  datos_struct[indice_entra].ruido = 1; ///// valores motor
  datos_struct[indice_entra].Presion = getPresionBmp();
  datos_struct[indice_entra].Alarma = 1;
  indice_entra++;
  Serial.print("saco valores de la struct ientra");
  Serial.println(indice_entra);
}


//**********************************************************//
//* SETUP  rtc                                           **//
//**********************************************************//
void setup_rtc_ntp(void)
{
  // init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  timestamp = time(NULL);
  rtc.setTime(timestamp + gmtOffset_sec);
}

//**********************************************************//
//* PIDE LA HORA AL SERVER O AL ESP32                     **//
//**********************************************************//
void pedir_lahora(void)
{
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("veo la hora del rtc interno ");
    timestamp = rtc.getEpoch() - gmtOffset_sec;
    timeinfo = rtc.getTimeStruct();
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  }
  else
  {
    Serial.print("NTP Time:");
    timestamp = time(NULL);
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  }

  return;
}
//**********************************************************//
//* wifi ap                                **//
//**********************************************************//
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Captive Portal Demo</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h1> EL PEPE </h1>
  <img src="data:image/jpeg;base64,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" alt="re duro" width="42" height="42" style="float:left">
  <br><br>
  <form action="/get">
    <br>
    SSID: <input type="text" name="ssid">
    <br>
    Password: <input type="text" name="password">
    <br>
    Token_tel: <input type="text" name="Token_tel">
    <br>
    Id_tel: <input type="number" name="Id_tel">
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";
void StartCaptivePortal(void);
class CaptiveRequestHandler : public AsyncWebHandler
{
  public:
    CaptiveRequestHandler() {}
    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request)
    {
      // request->addInterestingHeader("ANY");
      return true;
    }

    void handleRequest(AsyncWebServerRequest *request)
    {
      request->send_P(200, "text/html", index_html);
    }
};

void setupServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send_P(200, "text/html", index_html);
    Serial.println("Client Connected");
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    String inputMessage;
    String inputParam;

    if (request->hasParam("ssid")) {
      inputMessage = request->getParam("ssid")->value();
      inputParam = "ssid";
      ssid = inputMessage;
      Serial.println(inputMessage);
      valid_ssid_received = true;
    }

    if (request->hasParam("password")) {
      inputMessage = request->getParam("password")->value();
      inputParam = "password";
      password = inputMessage;
      Serial.println(inputMessage);
      valid_password_received = true;
    }


    if (request->hasParam("Token_tel")) {
      inputMessage = request->getParam("Token_tel")->value();
      inputParam = "Token_tel";
      Token_tel = inputMessage;
      Serial.println(inputMessage);
      valid_password_received = true;
    }

    if (request->hasParam("Id_tel")) {
      inputMessage = request->getParam("Id_tel")->value();
      inputParam = "Id_tel";
      Id_tel = inputMessage;
      Serial.println(inputMessage);
      valid_password_received = true;
    }

    request->send(200, "text/html", "The values entered by you have been successfully sent to the device. It will now attempt WiFi connection");
  });
}

void WiFiSoftAPSetup()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP("WIFI-meca-TILIN");///puede agregarle el nombre del grupo
  Serial.print(F("AP IP address: "));
  Serial.println(WiFi.softAPIP());
}

void WiFiStationSetup(String rec_ssid, String rec_password, String rec_Id_tel, String rec_Token_tel)
{
  wifi_timeout = false;
  WiFi.mode(WIFI_STA);

  rec_ssid.toCharArray(ap_ssid, rec_ssid.length() + 1);
  rec_password.toCharArray(ap_password, rec_password.length() + 1);
  rec_Token_tel.toCharArray(ap_token, rec_Token_tel.length() + 1);
  rec_Id_tel.toCharArray(ap_Id_tel, rec_Id_tel.length() + 1);

  // Serial.print("Received SSID: "); Serial.println(ap_ssid);
  // Serial.print("And password: "); Serial.println(ap_password);
  // Serial.print("And id: "); Serial.println(rec_Id_tel);
  // Serial.print("And token: ");Serial.println(rec_Token_tel);
  WiFi.begin(ap_ssid, ap_password);

  uint32_t t1 = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(2000);
    Serial.print(F("."));
    if (millis() - t1 > 50000) // 50 seconds elapsed connecting to WiFi
    {
      // Serial.println();
      Serial.println(F("Timeout connecting to WiFi. The SSID and Password seem incorrect."));
      valid_ssid_received = false;
      valid_password_received = false;
      is_setup_done = false;
      preferences.putBool("is_setup_done", is_setup_done);

      StartCaptivePortal();
      wifi_timeout = true;
      break;
    }
  }
  if (!wifi_timeout)
  {
    is_setup_done = true;
    // Serial.println("");
    Serial.print(F("WiFi connected to: "));
    Serial.println(rec_ssid);
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
    preferences.putBool("is_setup_done", is_setup_done);
    preferences.putString("rec_ssid", rec_ssid);
    preferences.putString("rec_password", rec_password);
    preferences.putString("rec_Token_tel", rec_Token_tel);
    preferences.putString("rec_Id_tel", rec_Id_tel);
  }
}

void StartCaptivePortal()
{
  Serial.println(F("Setting up AP Mode"));
  WiFiSoftAPSetup();
  Serial.println(F("Setting up Async WebServer"));
  setupServer();
  Serial.println(F("Starting DNS Server"));
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); // only when requested from AP
  server.begin();
  dnsServer.processNextRequest();
}

void ap_wifisetup()
{
  Serial.println();
  preferences.begin("my-pref", false);
  // preferences.clear();//////////////////////////////////linea de codigo para borrar la eeprom
  is_setup_done = preferences.getBool("is_setup_done", false);
  ssid = preferences.getString("rec_ssid", "Sample_SSID");
  ssid.toCharArray(ap_ssid, ssid.length() + 1);

  password = preferences.getString("rec_password", "abcdefgh");
  password.toCharArray(ap_password, password.length() + 1);

  Token_tel = preferences.getString("rec_Token_tel", "abcdefgh");
  Token_tel.toCharArray(ap_token, Token_tel.length() + 1);

  Id_tel = preferences.getString("rec_Id_tel", "abcdefgh");
  Id_tel.toCharArray(ap_Id_tel, Id_tel.length() + 1);
  if (!is_setup_done)
  {
    StartCaptivePortal();
  }
  else
  {
    // Serial.println("Using saved SSID and Password to attempt WiFi Connection!");
    Serial.print(F("Saved SSID is "));
    Serial.println(ssid);
    Serial.print(F("Saved Password is "));
    Serial.println(password);
    Serial.print(F("Saved token is "));
    Serial.println(Token_tel);
    Serial.print(F("Saved id is "));
    Serial.println(Id_tel);

    WiFiStationSetup(ssid, password, Id_tel, Token_tel);
  }

  while (!is_setup_done)
  {
    dnsServer.processNextRequest();
    delay(10);
    if (valid_ssid_received && valid_password_received)
    {
      Serial.println(F("Attempting WiFi Connection!"));
      WiFiStationSetup(ssid, password, Id_tel, Token_tel);
    }
  }
}

////////////////////////////////////////////////////////////////////
/////////////SETUP/////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  while (1) {}
  }
  /////declaro pines digitales
  setupmqtt();
  //Setup de time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  ap_wifisetup();
  setup_rtc_ntp();

  ledcSetup(channel, freq, resolution);  // Set up PWM channel
  ledcAttachPin(PIN_BUZZER, channel);                      // Attach channel to pin

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MQ5, INPUT_PULLUP);
  pinMode(PIN_MQ7, INPUT_PULLUP);
  pinMode(PIN_TEMT6000, INPUT_PULLUP);

  timerDisplay = timerBegin(2, 80, true);
  timerAttachInterrupt(timerDisplay, &onTimerDisplay, true);
  timerAlarmWrite(timerDisplay, TIMER_DISPLAY, true);
  timerAlarmEnable(timerDisplay);
}///////////////////////////////////////////////////



void loop() {
  //pedir_lahora();
  displayoled();
  alarma();

  //int temp = getTemperaturaBmp();
  //displayText(String(temp));

  /*now = millis();
  if (now - lastMeasure1 > interval_envio) {    ////envio el doble de lectura por si falla algun envio
    lastMeasure1 = now;/// cargo el valor actual de millis
    fun_envio_mqtt();///envio los valores por mqtt
  }
  if (now - lastMeasure2 > interval_leeo) {
    lastMeasure2 = now;/// cargo el valor actual de millis
    fun_entra(); ///ingreso los valores a la cola struct
  }*/

}
