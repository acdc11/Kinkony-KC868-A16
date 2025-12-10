/*
  KC868-A16 MASTER – Multi SID v21a (v20)
  Zgodnie z ustaleniami:
    - Bez zmian funkcjonalności istniejącej logiki (ETH, WiFi, MQTT, ModbusTCP/RTU, I2C, /io, /config, /mqtt/repub, /io/diag).
    - DOPISANE: Wyłącznie jedna nowa zakładka WWW: Modbus TCP Diagnostics (endpoint: /modbus/tcp/diag).
      Zawiera szczegółową (tekstową) dokumentację konfiguracji, komunikacji oraz składni poleceń do obsługi
      współdzielonej tablicy rejestrów falownika Delta ME300 (ModbusRTU) poprzez ModbusTCP (HREG/IREG/COIL/ISTS).
    - Brak uproszczeń. Brak zmian runtime ani w mapowaniu rejestrów – tylko strona informacyjna + link w nawigacji.

  Uwierzytelnianie: admin / darol177
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ETH.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ModbusIP_ESP8266.h>
#include <Wire.h>
#include <PCF8574.h>

// ========== Stałe sieciowe ==========
#ifndef RS485_RX_PIN
#define RS485_RX_PIN 16
#endif
#ifndef RS485_TX_PIN
#define RS485_TX_PIN 13
#endif
static_assert(RS485_RX_PIN==16,"KC868-A16: RS485_RX_PIN must be 16");
static_assert(RS485_TX_PIN==13,"KC868-A16: RS485_TX_PIN must be 13");

static const IPAddress DEF_ETH_IP (192,168,1,58);
static const IPAddress DEF_ETH_GW (192,168,1,1);
static const IPAddress DEF_ETH_SN (255,255,255,0);
static const IPAddress DEF_ETH_DNS(194,204,152,34);

static const char*     DEF_AP_SSID = "KINCONY_WIFI";
static const char*     DEF_AP_PASS = "kincony";
static const IPAddress DEF_AP_IP   (192,168,50,1);
static const IPAddress DEF_AP_SN   (255,255,255,0);

// MQTT
static const char* MQTT_HOST = "192.168.1.185";
static const int   MQTT_PORT = 1883;
static const char* MQTT_USER = "service";
static const char* MQTT_PASS = "11223344";
static const bool  MQTT_DO_DELTA = true;
static const bool  MQTT_PUBLISH_FULL_STATE = true;

// Obiekty
WebServer  server(80);
const char* www_realm = "KC868-A16 Admin";
ModbusIP   mbTCP;
WiFiClient _mqttNet;
PubSubClient mqtt(_mqttNet);
Preferences prefs;

// I2C / PCF8574
PCF8574 pcf_OUT1(0x24);
PCF8574 pcf_OUT2(0x25);
PCF8574 pcf_IN1 (0x22);
PCF8574 pcf_IN2 (0x21);
bool has_OUT1=false, has_OUT2=false, has_IN1=false, has_IN2=false;

// E16 (opcjonalne – pozostawione miejsce)
enum class E16Type : uint8_t { NONE=0, PCF8575, PCF8574_DUAL };
struct E16State {
  E16Type type=E16Type::NONE;
  uint8_t addr_pcf8575=0;
  PCF8574* pcf_a=nullptr;
  PCF8574* pcf_b=nullptr;
  bool present=false;
  uint16_t shadow=0xFFFF;
} e16;

// System status
struct SystemStatus {
  bool eth_connected=false;
  bool ap_active=false;
  bool sta_active=false;
  bool mqtt_connected=false;
  bool i2c_initialized=false;

  uint16_t di[16]={0};
  uint16_t dout[32]={0};
  uint16_t ai[4]={0};

  uint32_t last_io_pub=0;
  uint32_t last_mb_pub=0;
} systemStatus;

// Konfiguracja trwała
struct NetCfg {
  bool eth_dhcp=false;
  IPAddress eth_ip, eth_gw, eth_sn, eth_dns;

  bool wifi_ap=false;          // domyślnie STA jeśli brak klucza
  IPAddress ap_ip=DEF_AP_IP, ap_sn=DEF_AP_SN;
  String ap_ssid=DEF_AP_SSID, ap_pass=DEF_AP_PASS;

  String sta_ssid="", sta_pass="";
  uint16_t sta_fb_sec=30;      // fallback STA->AP po N sekundach bez IP
} netCfg;

struct TCPShadow { bool enabled=true; uint16_t port=502; } tcpCfg;
struct RTUShadow { uint32_t baud=9600; uint8_t parity=0; uint16_t pollMs=500; } rtuCfg;

// Append eksport (falowniki – zachowane)
extern "C" void     inverter_master_begin();
extern "C" bool     inverter_rtu_apply(uint8_t sid_unused, uint32_t baud, uint8_t parity, uint16_t pollMs);
extern "C" uint32_t inverter_get_last_state_pub();
extern "C" uint32_t inverter_get_last_decode_pub();

// Utils
static String ipToStr(const IPAddress& ip){
  return String(ip[0])+"."+String(ip[1])+"."+String(ip[2])+"."+String(ip[3]);
}
static bool strToIP(const String& s, IPAddress& out){
  IPAddress t; if(t.fromString(s)){ out=t; return true;} return false;
}

// WiFi events
static uint32_t sta_connect_start_ms=0;
static bool wifi_mode_pending_apply=false;
static bool wifi_target_ap=true;

static void onWiFiEvent(WiFiEvent_t event){
  switch(event){
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname("kc868-a16");
      Serial.println("[ETH] Start"); break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] LinkUp"); break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("[ETH] IP=%s MAC=%s Speed=%dMbps Duplex=%s\n",
        ETH.localIP().toString().c_str(),
        ETH.macAddress().c_str(),
        ETH.linkSpeed(),
        ETH.fullDuplex()?"Full":"Half");
      ETH.setDefault();
      systemStatus.eth_connected=true; break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] LinkDown"); systemStatus.eth_connected=false; break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[ETH] Stop"); systemStatus.eth_connected=false; break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WiFi STA] Connected (awaiting IP)"); break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi STA] IP=%s\n", WiFi.localIP().toString().c_str());
      systemStatus.sta_active=true; sta_connect_start_ms=0; break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi STA] Disconnected");
      systemStatus.sta_active=false;
      if(!netCfg.wifi_ap && sta_connect_start_ms==0) sta_connect_start_ms=millis();
      break;
    default: break;
  }
}

// Load/save
static void loadCfg(){
  prefs.begin("kc868cfg", true);
  IPAddress t;
  t.fromString(prefs.getString("eth_ip",  ipToStr(DEF_ETH_IP)));  netCfg.eth_ip=t;
  t.fromString(prefs.getString("eth_gw",  ipToStr(DEF_ETH_GW)));  netCfg.eth_gw=t;
  t.fromString(prefs.getString("eth_sn",  ipToStr(DEF_ETH_SN)));  netCfg.eth_sn=t;
  t.fromString(prefs.getString("eth_dns", ipToStr(DEF_ETH_DNS))); netCfg.eth_dns=t;

  netCfg.wifi_ap  = prefs.getBool("wifi_ap", false); // domyślnie STA
  netCfg.ap_ssid  = prefs.getString("ap_ssid", DEF_AP_SSID);
  netCfg.ap_pass  = prefs.getString("ap_pass", DEF_AP_PASS);
  t.fromString(prefs.getString("ap_ip", ipToStr(DEF_AP_IP))); netCfg.ap_ip=t;
  t.fromString(prefs.getString("ap_sn", ipToStr(DEF_AP_SN))); netCfg.ap_sn=t;

  netCfg.sta_ssid = prefs.getString("sta_ssid", "");
  netCfg.sta_pass = prefs.getString("sta_pass", "");
  netCfg.sta_fb_sec = prefs.getUShort("sta_fb_sec", 30);
  if(netCfg.sta_fb_sec<5) netCfg.sta_fb_sec=5;
  if(netCfg.sta_fb_sec>300) netCfg.sta_fb_sec=300;

  tcpCfg.enabled = prefs.getBool("tcp_en", true);
  tcpCfg.port    = prefs.getUShort("tcp_port", 502);
  prefs.end();

  prefs.begin("invrtu", true);
  rtuCfg.baud   = prefs.getULong("baud",9600);
  rtuCfg.parity = prefs.getUChar("par",0);
  rtuCfg.pollMs = prefs.getUShort("poll",500);
  prefs.end();
  rtuCfg.pollMs=constrain((int)rtuCfg.pollMs,100,5000);
}
static void saveNetCfg(){
  prefs.begin("kc868cfg", false);
  prefs.putString("eth_ip",  ipToStr(netCfg.eth_ip));
  prefs.putString("eth_gw",  ipToStr(netCfg.eth_gw));
  prefs.putString("eth_sn",  ipToStr(netCfg.eth_sn));
  prefs.putString("eth_dns", ipToStr(netCfg.eth_dns));
  prefs.putBool("wifi_ap", netCfg.wifi_ap);
  prefs.putString("ap_ssid", netCfg.ap_ssid);
  prefs.putString("ap_pass", netCfg.ap_pass);
  prefs.putString("ap_ip", ipToStr(netCfg.ap_ip));
  prefs.putString("ap_sn", ipToStr(netCfg.ap_sn));
  prefs.putString("sta_ssid", netCfg.sta_ssid);
  prefs.putString("sta_pass", netCfg.sta_pass);
  prefs.putUShort("sta_fb_sec", netCfg.sta_fb_sec);
  prefs.putBool("tcp_en", tcpCfg.enabled);
  prefs.putUShort("tcp_port", tcpCfg.port);
  prefs.end();
}
static void saveRTUShadow(){
  prefs.begin("invrtu", false);
  prefs.putULong("baud", rtuCfg.baud);
  prefs.putUChar("par",  rtuCfg.parity);
  prefs.putUShort("poll",rtuCfg.pollMs);
  prefs.end();
}

// ETH
static void setupETH(){
  WiFi.onEvent(onWiFiEvent);
  const eth_phy_type_t   TYPE=ETH_PHY_LAN8720;
  const int32_t          ADDR=0;
  const int              MDC=23;
  const int              MDIO=18;
  const int              PWR=-1;
  const eth_clock_mode_t CLK=ETH_CLOCK_GPIO17_OUT;
  if(!ETH.begin(TYPE,ADDR,MDC,MDIO,PWR,CLK))
    Serial.println("[ETH] begin failed");
  else
    Serial.println("[ETH] begin OK");
  ETH.config(netCfg.eth_ip, netCfg.eth_gw, netCfg.eth_sn, netCfg.eth_dns);
}

// WiFi mode switch
static void applyWifiMode(bool ap){
  Serial.printf("[WiFi] apply -> %s\n", ap?"AP":"STA");
  WiFi.disconnect(true);
  delay(100);
  if(ap){
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(netCfg.ap_ip, netCfg.ap_ip, netCfg.ap_sn);
    if(WiFi.softAP(netCfg.ap_ssid.c_str(), netCfg.ap_pass.c_str())){
      systemStatus.ap_active=true;
      systemStatus.sta_active=false;
      Serial.printf("[AP] IP=%s\n", WiFi.softAPIP().toString().c_str());
    } else {
      systemStatus.ap_active=false;
      Serial.println("[AP] start failed");
    }
  } else {
    WiFi.mode(WIFI_STA);
    systemStatus.ap_active=false;
    systemStatus.sta_active=false;
    if(netCfg.sta_ssid.length()){
      WiFi.begin(netCfg.sta_ssid.c_str(), netCfg.sta_pass.c_str());
      sta_connect_start_ms=millis();
      Serial.printf("[STA] Connecting '%s' (DHCP)\n", netCfg.sta_ssid.c_str());
    } else {
      Serial.println("[STA] Empty SSID -> fallback AP");
      netCfg.wifi_ap=true; saveNetCfg();
      applyWifiMode(true);
    }
  }
}
static void setupWiFiInitial(){ applyWifiMode(netCfg.wifi_ap); }

// Modbus TCP (minimalna przestrzeń – identycznie jak wcześniej)
static void setupMBTCP(){
  if(!tcpCfg.enabled){ Serial.println("[ModbusTCP] Disabled"); return; }
  mbTCP.server(tcpCfg.port);
  for(int i=0;i<96;i++){ mbTCP.addIreg(i); mbTCP.addHreg(i); }
  for(int i=0;i<64;i++){ mbTCP.addCoil(i); mbTCP.addIsts(i); }
  Serial.printf("[ModbusTCP] Server %u ready\n", tcpCfg.port);
}

// I2C
static bool i2cPresent(uint8_t a){ Wire.beginTransmission(a); return Wire.endTransmission()==0; }
static void setupI2C(){
  Wire.begin(4,5);
  Wire.setClock(100000);
  delay(150);
  has_OUT1=pcf_OUT1.begin()||i2cPresent(0x24);
  has_OUT2=pcf_OUT2.begin()||i2cPresent(0x25);
  has_IN1 =pcf_IN1.begin() ||i2cPresent(0x22);
  has_IN2 =pcf_IN2.begin() ||i2cPresent(0x21);
  if(has_OUT1){ for(int i=0;i<8;i++){ pcf_OUT1.pinMode(i,OUTPUT); pcf_OUT1.digitalWrite(i,HIGH);} }
  if(has_OUT2){ for(int i=0;i<8;i++){ pcf_OUT2.pinMode(i,OUTPUT); pcf_OUT2.digitalWrite(i,HIGH);} }
  if(has_IN1 ){ for(int i=0;i<8;i++) pcf_IN1.pinMode(i,INPUT); }
  if(has_IN2 ){ for(int i=0;i<8;i++) pcf_IN2.pinMode(i,INPUT); }
  systemStatus.i2c_initialized=(has_OUT1||has_OUT2||has_IN1||has_IN2);
  analogReadResolution(12);
}

static void readInputs(){
  if(has_IN1){
    for(int i=0;i<8;i++){
      uint8_t v=pcf_IN1.digitalRead(i);
      systemStatus.di[i]=v?0:1;
      mbTCP.Ists(i,systemStatus.di[i]);
    }
  }
  if(has_IN2){
    for(int i=0;i<8;i++){
      uint8_t v=pcf_IN2.digitalRead(i);
      systemStatus.di[i+8]=v?0:1;
      mbTCP.Ists(i+8,systemStatus.di[i+8]);
    }
  }
  systemStatus.ai[0]=analogRead(32);
  systemStatus.ai[1]=analogRead(33);
  systemStatus.ai[2]=analogRead(34);
  systemStatus.ai[3]=analogRead(35);
}

static void updateOutputs(){
  for(int i=0;i<8;i++){
    bool new1=mbTCP.Coil(i);
    bool new2=mbTCP.Coil(i+8);
    bool prev1=systemStatus.dout[i];
    bool prev2=systemStatus.dout[i+8];
    if(has_OUT1) pcf_OUT1.digitalWrite(i, new1 ? LOW : HIGH);
    if(has_OUT2) pcf_OUT2.digitalWrite(i, new2 ? LOW : HIGH);
    systemStatus.dout[i]   = new1?1:0;
    systemStatus.dout[i+8] = new2?1:0;
    if(MQTT_DO_DELTA && mqtt.connected()){
      if(new1!=prev1){ String t="KINCONY/INOUT/do/"+String(i+1);  mqtt.publish(t.c_str(), new1?"1":"0", true); }
      if(new2!=prev2){ String t="KINCONY/INOUT/do/"+String(i+9);  mqtt.publish(t.c_str(), new2?"1":"0", true); }
    }
  }
  if(e16.present){
    uint16_t val=0xFFFF;
    for(int i=0;i<16;i++){
      bool on=mbTCP.Coil(16+i);
      if(on) val&=~(1<<i); else val|=(1<<i);
      systemStatus.dout[16+i]=on?1:0;
    }
    // e16WriteAll(val) – implementacja pozostaje jak wcześniej (opcjonalna)
  }
}

// MQTT
static uint32_t lastMqttAttempt=0;
static void mqttCallback(char* topic, byte* payload, unsigned int len){
  // Zostawiono puste – implementacja może być rozszerzona później
}
static void setupMQTT(){
  mqtt.setServer(MQTT_HOST,MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}
static void ensureMqtt(){
  if(mqtt.connected()) return;
  if(millis()-lastMqttAttempt<5000) return;
  lastMqttAttempt=millis();
  bool ok=strlen(MQTT_USER)? mqtt.connect("KC868-A16",MQTT_USER,MQTT_PASS):mqtt.connect("KC868-A16");
  systemStatus.mqtt_connected=ok;
  if(ok){
    mqtt.subscribe("KINCONY/INOUT/set");
    mqtt.subscribe("KINCONY/MODBUSRTU/set");
  }
}
static void publishIO(){
  if(!mqtt.connected()) return;
  String s;
  s.reserve(400);
  s="{\"digital_inputs\":[";
  for(int i=0;i<16;i++){ s+=systemStatus.di[i]; if(i<15) s+=","; }
  s+="],\"analog_inputs\":[";
  for(int i=0;i<4;i++){ s+=systemStatus.ai[i]; if(i<3) s+=","; }
  s+="],\"digital_outputs\":[";
  int maxOut=e16.present?32:16;
  for(int i=0;i<maxOut;i++){ s+=systemStatus.dout[i]; if(i<maxOut-1) s+=","; }
  s+="]}";
  mqtt.publish("KINCONY/INOUT/state", s.c_str(), false);
  systemStatus.last_io_pub=millis();
}
static void publishMB(){
  if(!mqtt.connected()) return;
  String j;
  j.reserve(300);
  j="{\"holding_registers\":[";
  for(int i=0;i<10;i++){ j+=mbTCP.Hreg(i); if(i<9) j+=","; }
  j+="],\"input_registers\":[";
  for(int i=0;i<10;i++){ j+=mbTCP.Ireg(i); if(i<9) j+=","; }
  j+="]}";
  mqtt.publish("KINCONY/MODBUSRTU/state", j.c_str(), false);
  systemStatus.last_mb_pub=millis();
}

// Auth
static bool requireAuth(){
  if(!server.authenticate("admin","darol177")){
    server.requestAuthentication(DIGEST_AUTH, www_realm);
    return false;
  }
  return true;
}

// WWW: Root
static void handleRoot(){
  if(!requireAuth()) return;
  String html;
  html.reserve(1800);
  html+="<!DOCTYPE html><html><head><meta charset='utf-8'><title>KC868-A16</title>";
  html+="<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html+="<style>body{font-family:Arial;margin:20px;background:#f5f5f5}";
  html+=".card{background:#fff;padding:16px;border-radius:12px;box-shadow:0 2px 6px rgba(0,0,0,.12)}";
  html+="a.btn{display:inline-block;padding:10px 14px;background:#1976d2;color:#fff;border-radius:8px;text-decoration:none;margin:4px}";
  html+=".hint{font-size:12px;color:#666;margin-top:4px}</style></head><body>";
  html+="<h2>KC868-A16 Control (Multi Inverter)</h2><div class='card'><p>";
  html+="<a class='btn' href='/inverter_master'>Inverter Panel</a>";
  html+="<a class='btn' href='/config'>Config</a>";
  html+="<a class='btn' href='/io'>I/O Panel</a>";
  html+="<a class='btn' href='/io/diag'>I/O Diagnostics</a>";
  html+="<a class='btn' href='/critical'>Critical</a>";
  html+="<a class='btn' href='/mqtt/repub'>MQTT Topics</a>";
  html+="<a class='btn' href='/mqtt/repub/ui'>MQTT Republish</a>";
  html+="<a class='btn' href='/modbus/tcp/diag'>Modbus TCP Diagnostics</a>";
  html+="</p><div class='hint'>/inverter_master dostarcza moduł append.</div>";
  html+="<p>ETH: ";
  html+=(systemStatus.eth_connected?ETH.localIP().toString():"(no link)");
  html+=" | AP: ";
  html+=(systemStatus.ap_active?WiFi.softAPIP().toString():"(inactive)");
  html+=" | STA: ";
  html+=(systemStatus.sta_active?WiFi.localIP().toString():"(inactive)");
  html+="</p>";
  html+="<p>WiFi Mode: ";
  html+= netCfg.wifi_ap ? "AP" : "STA";
  html+= " (fallback ";
  html+= String(netCfg.sta_fb_sec);
  html+= "s)</p>";
  html+="</div></body></html>";
  server.send(200,"text/html", html);
}

// WWW: Status
static void handleStatus(){
  String j;
  j.reserve(240);
  j="{\"eth\":"+String(systemStatus.eth_connected?"true":"false")+
    ",\"ap\":"+String(systemStatus.ap_active?"true":"false")+
    ",\"sta\":"+String(systemStatus.sta_active?"true":"false")+
    ",\"mqtt\":"+String(systemStatus.mqtt_connected?"true":"false")+
    ",\"i2c\":"+String(systemStatus.i2c_initialized?"true":"false")+
    ",\"ip_eth\":\""+ETH.localIP().toString()+"\""+
    ",\"ip_ap\":\""+(systemStatus.ap_active?WiFi.softAPIP().toString():"")+"\""+
    ",\"ip_sta\":\""+(systemStatus.sta_active?WiFi.localIP().toString():"")+"\""+
    ",\"wifi_mode\":\""+String(netCfg.wifi_ap?"AP":"STA")+"\""+
    ",\"sta_fb_sec\":"+String(netCfg.sta_fb_sec)+"}";
  server.send(200,"application/json", j);
}

// WWW: Critical (JSON)
static void handleCritical(){
  if(!requireAuth()) return;
  String j;
  j.reserve(700);
  j="{\"eth\":{\"ok\":"+String(systemStatus.eth_connected?"true":"false")+",\"ip\":\""+ETH.localIP().toString()+"\"}";
  j+=",\"ap\":{\"ok\":"+String(systemStatus.ap_active?"true":"false")+",\"ip\":\""+(systemStatus.ap_active?WiFi.softAPIP().toString():"")+"\"}";
  j+=",\"sta\":{\"ok\":"+String(systemStatus.sta_active?"true":"false")+",\"ip\":\""+(systemStatus.sta_active?WiFi.localIP().toString():"")+"\"}";
  j+=",\"mqtt\":{\"ok\":"+String(systemStatus.mqtt_connected?"true":"false")+"}";
  j+=",\"modbus_tcp\":{\"enabled\":"+String(tcpCfg.enabled?"true":"false")+",\"port\":"+String(tcpCfg.port)+"}";
  j+=",\"rs485\":{\"rx\":"+String(RS485_RX_PIN)+",\"tx\":"+String(RS485_TX_PIN)+"}";
  j+="}";
  server.send(200,"application/json", j);
}

// MQTT Republish UI
static void handleMqttRepubUI(){
  if(!requireAuth()) return;
  String html="<!DOCTYPE html><html><head><meta charset='utf-8'><title>MQTT Republish</title>"
              "<meta name='viewport' content='width=device-width,initial-scale=1'>"
              "<style>body{font-family:Arial;margin:16px}.card{background:#fff;padding:12px;border-radius:8px;margin-bottom:12px;border:1px solid #ddd}"
              ".btn{display:inline-block;padding:6px 10px;background:#1976d2;color:#fff;text-decoration:none;border-radius:6px;margin:4px}</style>"
              "</head><body><h2>MQTT Topics</h2>"
              "<div class='card'><h3>Publish</h3><ul>"
              "<li>KINCONY/INOUT/state <a class='btn' href='/mqtt/repub/publish?topic=inout'>Republish</a></li>"
              "<li>KINCONY/MODBUSRTU/state <a class='btn' href='/mqtt/repub/publish?topic=modbus'>Republish</a></li>"
              "<li>KINCONY/INVERTER/status <a class='btn' href='/mqtt/repub/publish?topic=inverter'>Republish</a></li>"
              "</ul></div>"
              "<div class='card'><h3>Manual publish</h3>"
              "<form method='POST' action='/mqtt/repub/set'>"
              "<label>Topic:<input name='topic' style='width:320px'></label><br><br>"
              "<label>Payload:<input name='payload' style='width:320px'></label><br><br>"
              "<button class='btn' type='submit'>Publish</button>"
              "</form></div>"
              "<p><a href='/'>Back</a></p></body></html>";
  server.send(200,"text/html", html);
}

// MQTT Topics doc + szablony /set pod treścią
static void handleMqttTopics(){
  if(!requireAuth()) return;
  const char* page="<!doctype html><html><head><meta charset='utf-8'><title>MQTT topics</title>"
                   "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                   "<style>body{font-family:Arial;margin:20px;background:#f5f5f5}.section{background:#fff;padding:16px;border-radius:10px;box-shadow:0 2px 5px rgba(0,0,0,.1);margin-bottom:12px}"
                   "table{border-collapse:collapse;width:100%;max-width:900px}th,td{border:1px solid #ddd;padding:8px;text-align:left;font-size:13px}"
                   "th{background:#f3f3f3}a.button{display:inline-block;padding:8px 12px;background:#1976d2;color:#fff;border-radius:6px;text-decoration:none}"
                   "pre{background:#0e1014;color:#e6edf3;padding:8px;border-radius:6px;overflow:auto}</style>"
                   "</head><body><div class='section'><h2>MQTT topics</h2><table>"
                   "<tr><th>Typ</th><th>Topic</th><th>Opis</th></tr>"
                   "<tr><td>publish</td><td>KINCONY/INVERTER/&lt;sid&gt;/state</td><td>Telemetria falownika</td></tr>"
                   "<tr><td>publish</td><td>KINCONY/INVERTER/&lt;sid&gt;/decode</td><td>Decode flags</td></tr>"
                   "<tr><td>publish</td><td>KINCONY/MODBUSRTU/state</td><td>Zrzut rejestrów</td></tr>"
                   "<tr><td>publish</td><td>KINCONY/INOUT/state</td><td>We/Wy</td></tr>"
                   "<tr><td>subscribe</td><td>KINCONY/INVERTER/&lt;sid&gt;/set</td><td>Sterowanie falownikiem</td></tr>"
                   "<tr><td>subscribe</td><td>KINCONY/INOUT/set</td><td>Ustaw wyjścia</td></tr>"
                   "<tr><td>subscribe</td><td>KINCONY/MODBUSRTU/set</td><td>Parametry RTU / wpisy HREG</td></tr>"
                   "</table>"
                   "<h3>Szablony /set</h3>"
                   "<pre>{\"outputs\":[1,0,...],\"coils\":[1,0,...]}</pre>"
                   "<pre>{\"rtu\":{\"sid\":1,\"baud\":9600,\"par\":\"8N1\",\"poll\":500},\"write_hreg\":[{\"addr\":0,\"value\":2}]}</pre>"
                   "<pre>{\"start\":true,\"dir\":\"fwd\",\"acc_set\":1,\"preset\":5,\"preset_off\":false,\"base_block\":false,\"reset\":false,\"freq\":50.00}</pre>"
                   "<p><a class='button' href='/'>Back</a></p></div></body></html>";
  server.send(200,"text/html", page);
}

// MQTT Republish actions
static void handleMqttRepubPublish(){
  if(!requireAuth()) return;
  String key=server.arg("topic");
  if(key=="inout"||key=="inout_state"){ publishIO(); server.send(200,"application/json","{\"ok\":\"inout_republished\"}"); return; }
  if(key=="modbus"||key=="modbus_state"){ publishMB(); server.send(200,"application/json","{\"ok\":\"modbus_republished\"}"); return; }
  if(key=="inverter"){
    if(!mqtt.connected()){ server.send(503,"application/json","{\"error\":\"mqtt_not_connected\"}"); return; }
    String js="{\"out_freq\":"+String(mbTCP.Ireg(1))+",\"out_curr\":"+String(mbTCP.Ireg(2))+",\"out_volt\":"+String(mbTCP.Ireg(3))+",\"dc_bus\":"+String(mbTCP.Ireg(5))+",\"status\":"+String(mbTCP.Ireg(0))+"}";
    mqtt.publish("KINCONY/INVERTER/status", js.c_str(), false);
    server.send(200,"application/json","{\"ok\":\"inverter_republished\"}");
    return;
  }
  server.send(400,"application/json","{\"error\":\"unknown_topic_key\"}");
}
static void handleMqttRepubSetPublish(){
  if(!requireAuth()) return;
  String topic=server.arg("topic");
  String payload=server.arg("payload");
  if(!topic.length()){ server.send(400,"application/json","{\"error\":\"missing_topic\"}"); return; }
  bool ok=mqtt.publish(topic.c_str(), payload.c_str());
  server.send(ok?200:500,"application/json", ok?"{\"ok\":\"published\"}":"{\"error\":\"publish_failed\"}");
}

// Config GET
static void handleConfigGet(){
  if(!requireAuth()) return;
  String html="<!DOCTYPE html><html><head><meta charset='utf-8'><title>KC868-A16 Configuration</title>"
             "<style>body{font-family:Arial;margin:20px}fieldset{margin-bottom:16px;padding:12px;border:1px solid #ccc;border-radius:8px}"
             "label{display:block;margin:6px 0 2px}input[type=text],select{width:260px;padding:6px}.btn{padding:8px 12px;background:#1976d2;color:#fff;border:none;border-radius:6px;cursor:pointer}</style>"
             "</head><body><h2>Configuration</h2><form method='POST' action='/config'>"
             "<fieldset><legend>Ethernet</legend>"
             "<label>IP</label><input type='text' name='eth_ip' value='%ETH_IP%'>"
             "<label>Subnet</label><input type='text' name='eth_sn' value='%ETH_SN%'>"
             "<label>Gateway</label><input type='text' name='eth_gw' value='%ETH_GW%'>"
             "<label>DNS</label><input type='text' name='eth_dns' value='%ETH_DNS%'>"
             "</fieldset>"
             "<fieldset><legend>WiFi AP</legend>"
             "<label>SSID</label><input type='text' name='ap_ssid' value='%AP_SSID%'>"
             "<label>Password</label><input type='text' name='ap_pass' value='%AP_PASS%'>"
             "<label>AP IP</label><input type='text' name='ap_ip' value='%AP_IP%'>"
             "<label>AP Subnet</label><input type='text' name='ap_sn' value='%AP_SN%'>"
             "</fieldset>"
             "<fieldset><legend>Modbus RTU (Global)</legend>"
             "<label>Baud</label><select name='rtu_baud'>"
             "<option %B9600% value='9600'>9600</option>"
             "<option %B19200% value='19200'>19200</option>"
             "<option %B38400% value='38400'>38400</option>"
             "<option %B57600% value='57600'>57600</option>"
             "<option %B115200% value='115200'>115200</option>"
             "</select>"
             "<label>Parity</label><select name='rtu_par'>"
             "<option %P0% value='0'>8N1</option>"
             "<option %P1% value='1'>8E1</option>"
             "<option %P2% value='2'>8O1</option>"
             "</select>"
             "<label>Poll [ms]</label><input type='text' name='rtu_poll' value='%RTU_POLL%'>"
             "</fieldset>"
             "<fieldset><legend>Modbus TCP</legend>"
             "<label>Enabled</label><select name='tcp_en'><option %TCP_ON% value='1'>ON</option><option %TCP_OFF% value='0'>OFF</option></select>"
             "<label>Port</label><input type='text' name='tcp_port' value='%TCP_PORT%'>"
             "</fieldset>"
             "<fieldset><legend>WiFi Mode & STA (DHCP + Fallback)</legend>"
             "<label>Mode</label><select name='wifi_mode'>"
             "<option value='AP' %WIFI_AP_MODE%>Access Point (AP)</option>"
             "<option value='STA' %WIFI_STA_MODE%>Station (STA)</option>"
             "</select>"
             "<label>STA SSID</label><input type='text' name='sta_ssid' value='%STA_SSID%'>"
             "<label>STA Password</label><input type='text' name='sta_pass' value='%STA_PASS%'>"
             "<label>STA Fallback [s]</label><input type='text' name='sta_fb_sec' value='%STA_FB_SEC%'>"
             "<div style='font-size:12px;color:#555;margin-top:6px'>Jeśli brak IP w STA przez podany czas – automatyczny powrót do AP.</div>"
             "</fieldset>"
             "<p><button class='btn' type='submit'>Save & Apply (no restart)</button> <a class='btn' href='/'>Back</a></p>"
             "</form></body></html>";

  html.replace("%ETH_IP%",ipToStr(netCfg.eth_ip));
  html.replace("%ETH_SN%",ipToStr(netCfg.eth_sn));
  html.replace("%ETH_GW%",ipToStr(netCfg.eth_gw));
  html.replace("%ETH_DNS%",ipToStr(netCfg.eth_dns));
  html.replace("%AP_SSID%", netCfg.ap_ssid);
  html.replace("%AP_PASS%", netCfg.ap_pass);
  html.replace("%AP_IP%", ipToStr(netCfg.ap_ip));
  html.replace("%AP_SN%", ipToStr(netCfg.ap_sn));
  html.replace("%RTU_POLL%", String(rtuCfg.pollMs));
  html.replace("%B9600%", rtuCfg.baud==9600?"selected":"");
  html.replace("%B19200%", rtuCfg.baud==19200?"selected":"");
  html.replace("%B38400%", rtuCfg.baud==38400?"selected":"");
  html.replace("%B57600%", rtuCfg.baud==57600?"selected":"");
  html.replace("%B115200%", rtuCfg.baud==115200?"selected":"");
  html.replace("%P0%", rtuCfg.parity==0?"selected":"");
  html.replace("%P1%", rtuCfg.parity==1?"selected":"");
  html.replace("%P2%", rtuCfg.parity==2?"selected":"");
  html.replace("%TCP_ON%", tcpCfg.enabled?"selected":"");
  html.replace("%TCP_OFF%", tcpCfg.enabled?"":"selected");
  html.replace("%TCP_PORT%", String(tcpCfg.port));
  html.replace("%WIFI_AP_MODE%", netCfg.wifi_ap?"selected":"");
  html.replace("%WIFI_STA_MODE%", netCfg.wifi_ap?"":"selected");
  html.replace("%STA_SSID%", netCfg.sta_ssid);
  html.replace("%STA_PASS%", netCfg.sta_pass);
  html.replace("%STA_FB_SEC%", String(netCfg.sta_fb_sec));

  server.send(200,"text/html", html);
}

// Config POST
static void handleConfigPost(){
  if(!requireAuth()) return;
  IPAddress ip;
  if(strToIP(server.arg("eth_ip"), ip))  netCfg.eth_ip=ip;
  if(strToIP(server.arg("eth_sn"), ip))  netCfg.eth_sn=ip;
  if(strToIP(server.arg("eth_gw"), ip))  netCfg.eth_gw=ip;
  if(strToIP(server.arg("eth_dns"),ip))  netCfg.eth_dns=ip;

  String mode = server.arg("wifi_mode");
  if(mode.length()){ mode.toUpperCase(); netCfg.wifi_ap = (mode!="STA"); }
  if(server.arg("ap_ssid").length()) netCfg.ap_ssid=server.arg("ap_ssid");
  if(server.arg("ap_pass").length()) netCfg.ap_pass=server.arg("ap_pass");
  if(strToIP(server.arg("ap_ip"), ip)) netCfg.ap_ip=ip;
  if(strToIP(server.arg("ap_sn"), ip)) netCfg.ap_sn=ip;
  if(server.arg("sta_ssid").length()) netCfg.sta_ssid=server.arg("sta_ssid");
  if(server.arg("sta_pass").length()) netCfg.sta_pass=server.arg("sta_pass");
  int fb=server.arg("sta_fb_sec").toInt(); if(fb>=5 && fb<=300) netCfg.sta_fb_sec=(uint16_t)fb;

  uint32_t baud=(uint32_t)server.arg("rtu_baud").toInt(); if(baud) rtuCfg.baud=baud;
  int par=server.arg("rtu_par").toInt(); if(par>=0 && par<=2) rtuCfg.parity=par;
  int poll=server.arg("rtu_poll").toInt(); if(poll>=100 && poll<=5000) rtuCfg.pollMs=poll;
  tcpCfg.enabled = server.arg("tcp_en")=="1";
  int p=server.arg("tcp_port").toInt(); if(p>=1 && p<=65535) tcpCfg.port=(uint16_t)p;

  saveNetCfg(); saveRTUShadow();
  inverter_rtu_apply(0, rtuCfg.baud, rtuCfg.parity, rtuCfg.pollMs);

  // Dynamiczne przełączenie WiFi (bez restartu)
  wifi_mode_pending_apply=true;
  wifi_target_ap=netCfg.wifi_ap;

  server.send(200,"application/json",
    "{\"ok\":true,"
    "\"wifi_mode\":\""+String(netCfg.wifi_ap?"AP":"STA")+"\","
    "\"sta_fb_sec\":"+String(netCfg.sta_fb_sec)+","
    "\"rtu_applied\":true}");
}

// IO PANEL (POPRAWIONA wersja – credentials:'include', JSON response)
static void handleIOPage(){
  if(!requireAuth()) return;
  String html="<!DOCTYPE html><html><head><meta charset='utf-8'><title>I/O Panel</title>"
              "<meta name='viewport' content='width=device-width,initial-scale=1'>"
              "<style>body{font-family:Arial;margin:20px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}"
              ".card{border:1px solid #ccc;border-radius:8px;padding:12px}.chip{display:inline-block;padding:4px 8px;border-radius:6px;margin:2px;border:1px solid #999}"
              ".on{background:#c8f7c5;border-color:#27ae60}.off{background:#ffd1d1;border-color:#c0392b}.btn{padding:6px 10px;margin:2px;border:1px solid #1976d2;background:#1976d2;color:#fff;border-radius:6px;cursor:pointer}</style>"
              "<script>"
              "async function loadIO(){"
              " try{const s=await fetch('/io/state',{credentials:'include'}).then(r=>r.json());"
              " const di=document.getElementById('di');di.innerHTML='';"
              " for(let i=0;i<16;i++){const v=s.di[i];di.innerHTML+=`<span class='chip ${v?'on':'off'}'>DI${(i+1).toString().padStart(2,'0')}:${v}</span>`}"
              " const ai=document.getElementById('ai');ai.innerHTML='';for(let i=0;i<4;i++){ai.innerHTML+=`<div>AI${i+1}: ${s.ai[i]}</div>`}"
              " const dox=document.getElementById('do');dox.innerHTML='';for(let i=0;i<(s.has_e16?32:16);i++){const v=s.do[i];dox.innerHTML+=`<button id='btn${i}' class='btn' onclick='tog(${i})'>Y${(i+1).toString().padStart(2,'0')} => ${v?'ON':'OFF'}</button>`}"
              " }catch(e){console.error('IO load error',e);} setTimeout(loadIO,1000);}"
              "async function tog(ch){"
              " const b=document.getElementById('btn'+ch); if(!b) return;"
              " try{const r=await fetch('/io/set?ch='+ch+'&v=toggle',{credentials:'include'});"
              " if(r.status==401){console.warn('Auth 401 /io/set');return;} const j=await r.json();"
              " if(j.ok){b.innerHTML=`Y${(ch+1).toString().padStart(2,'0')} => ${j.state==1?'ON':'OFF'}`} else {console.warn('err',j);} }catch(e){console.error('toggle error',e);}"
              "}"
              "window.onload=loadIO;"
              "</script></head><body><h2>I/O Panel</h2>"
              "<div class='grid'><div class='card'><h3>Digital Inputs</h3><div id='di'></div></div>"
              "<div class='card'><h3>Analog Inputs</h3><div id='ai'></div></div>"
              "<div class='card'><h3>Digital Outputs</h3><div id='do'></div></div>"
              "</div><p><a href='/'>Back</a></p></body></html>";
  server.send(200,"text/html", html);
}
static void handleIOState(){
  String j;
  j.reserve(600);
  j="{\"di\":[";
  for(int i=0;i<16;i++){ j+=systemStatus.di[i]; if(i<15) j+=","; }
  j+="],\"ai\":[";
  for(int i=0;i<4;i++){ j+=systemStatus.ai[i]; if(i<3) j+=","; }
  j+="],\"do\":[";
  int maxOut=e16.present?32:16;
  for(int i=0;i<maxOut;i++){ j+=systemStatus.dout[i]; if(i<maxOut-1) j+=","; }
  j+="],\"has_e16\":";
  j+=(e16.present?"true":"false");
  j+="}";
  server.send(200,"application/json", j);
}
static void handleIOSet(){
  if(!requireAuth()) return;
  if(!server.hasArg("ch") || !server.hasArg("v")){
    server.send(400,"application/json","{\"error\":\"missing_params\"}");
    return;
  }
  int ch=server.arg("ch").toInt();
  String v=server.arg("v");
  int maxOut=e16.present?32:16;
  if(ch<0 || ch>=maxOut){
    server.send(400,"application/json","{\"error\":\"invalid_channel\"}");
    return;
  }
  bool cur=systemStatus.dout[ch];
  bool nv=(v=="toggle")? !cur : (v=="1");
  mbTCP.Coil(ch,nv);
  systemStatus.dout[ch]=nv?1:0; // optimistic immediate local update
  Serial.printf("[IO SET] ch=%d cur=%d new=%d\n", ch, cur, nv);
  server.send(200,"application/json", String("{\"ok\":true,\"ch\":")+ch+",\"state\":"+(nv?"1":"0")+"}");
}

// DODANE: /io/diag – diagnostyka PCF i mapowanie cewek/zapisów (bez naruszania struktury)
static void handleIODiag(){
  if(!requireAuth()) return;
  bool raw = server.hasArg("raw") && server.arg("raw")=="1";

  uint8_t in1_bits=0xFF, in2_bits=0xFF, out1_bits=0xFF, out2_bits=0xFF;
  if(has_IN1){ for(int i=0;i<8;i++){ uint8_t v=pcf_IN1.digitalRead(i); if(v) in1_bits |= (1<<i); else in1_bits &= ~(1<<i);} }
  if(has_IN2){ for(int i=0;i<8;i++){ uint8_t v=pcf_IN2.digitalRead(i); if(v) in2_bits |= (1<<i); else in2_bits &= ~(1<<i);} }
  if(has_OUT1){ for(int i=0;i<8;i++){ if(systemStatus.dout[i]) out1_bits &= ~(1<<i); else out1_bits |= (1<<i);} }
  if(has_OUT2){ for(int i=0;i<8;i++){ if(systemStatus.dout[i+8]) out2_bits &= ~(1<<i); else out2_bits |= (1<<i);} }

  int maxOut=e16.present?32:16;
  String j;
  j.reserve(900);
  j="{\"i2c_present\":{\"OUT1\":";
  j+=(has_OUT1?"true":"false");
  j+=",\"OUT2\":";
  j+=(has_OUT2?"true":"false");
  j+=",\"IN1\":";
  j+=(has_IN1?"true":"false");
  j+=",\"IN2\":";
  j+=(has_IN2?"true":"false");
  j+="},\"pcf_raw\":{\"IN1_bits\":\"0b";
  for(int i=7;i>=0;i--) j+=(in1_bits&(1<<i))?'1':'0';
  j+="\",\"IN2_bits\":\"0b";
  for(int i=7;i>=0;i--) j+=(in2_bits&(1<<i))?'1':'0';
  j+="\",\"OUT1_shadow\":\"0b";
  for(int i=7;i>=0;i--) j+=(out1_bits&(1<<i))?'1':'0';
  j+="\",\"OUT2_shadow\":\"0b";
  for(int i=7;i>=0;i--) j+=(out2_bits&(1<<i))?'1':'0';
  j+="\"},\"map\":{\"coils\":[";
  for(int i=0;i<maxOut;i++){ j+=(mbTCP.Coil(i)?"1":"0"); if(i<maxOut-1) j+=","; }
  j+="],\"digital_outputs\":[";
  for(int i=0;i<maxOut;i++){ j+=String(systemStatus.dout[i]); if(i<maxOut-1) j+=","; }
  j+="],\"digital_inputs\":[";
  for(int i=0;i<16;i++){ j+=String(systemStatus.di[i]); if(i<15) j+=","; }
  j+="]},\"notes\":\"INx_bits: 1=HIGH(pull-up, nieaktywne), 0=LOW(aktywny). OUTx_shadow: 0=LOW(ON), 1=HIGH(OFF). Coils=mapa ModbusTCP; DO=shadow logiczny.\"}";

  if(raw){ server.send(200,"application/json", j); return; }

  String html="<!doctype html><html><head><meta charset='utf-8'><title>I/O Diagnostics</title>"
              "<meta name='viewport' content='width=device-width,initial-scale=1'>"
              "<style>:root{--bg:#f5f7fb;--card:#fff;--line:#e3e7ef}body{font-family:Arial;margin:0;background:var(--bg);color:#222}"
              ".wrap{max-width:1100px;margin:0 auto;padding:18px}.card{background:var(--card);padding:16px;border-radius:12px;box-shadow:0 2px 6px rgba(0,0,0,.08);margin-bottom:14px}"
              "table{width:100%;border-collapse:collapse}th,td{padding:8px 10px;border-bottom:1px solid var(--line);text-align:left;font-size:13px}"
              "code,pre{background:#0e1014;color:#e6edf3;padding:10px;border-radius:8px;overflow:auto} .mono{font-family:monospace}</style>"
              "</head><body><div class='wrap'><h1>I/O Diagnostics (PCF8574 + ModbusTCP maps)</h1>"
              "<div class='card'><h3>Presence</h3><table><tr><th>Chip</th><th>Present</th></tr>"
              "<tr><td>PCF8574 OUT1 (0x24)</td><td>"+String(has_OUT1?"YES":"NO")+"</td></tr>"
              "<tr><td>PCF8574 OUT2 (0x25)</td><td>"+String(has_OUT2?"YES":"NO")+"</td></tr>"
              "<tr><td>PCF8574 IN1 (0x22)</td><td>"+String(has_IN1?"YES":"NO")+"</td></tr>"
              "<tr><td>PCF8574 IN2 (0x21)</td><td>"+String(has_IN2?"YES":"NO")+"</td></tr>"
              "</table></div>"
              "<div class='card'><h3>Raw pins</h3>"
              "<p class='mono'>IN1 bits: ";
  html+="0b"; for(int i=7;i>=0;i--) html+=(in1_bits&(1<<i))?'1':'0';
  html+="<br>IN2 bits: 0b"; for(int i=7;i>=0;i--) html+=(in2_bits&(1<<i))?'1':'0';
  html+="<br>OUT1 shadow: 0b"; for(int i=7;i>=0;i--) html+=(out1_bits&(1<<i))?'1':'0';
  html+="<br>OUT2 shadow: 0b"; for(int i=7;i>=0;i--) html+=(out2_bits&(1<<i))?'1':'0';
  html+="</p><p>Legenda: IN=1 oznacza stan wysoki (nieaktywne, pull-up), IN=0 oznacza niski (aktywny). OUT shadow: 0=LOW(ON), 1=HIGH(OFF) – przekaźniki active-low.</p></div>";
  html+="<div class='card'><h3>ModbusTCP map</h3><table><tr><th>Index</th><th>Coil</th><th>DO shadow</th></tr>";
  for(int i=0;i<maxOut;i++){
    html+="<tr><td>"+String(i)+"</td><td>"+String(mbTCP.Coil(i)?"1":"0")+"</td><td>"+String(systemStatus.dout[i])+"</td></tr>";
  }
  html+="</table></div><div class='card'><h3>Digital inputs (decoded)</h3><p class='mono'>";
  for(int i=0;i<16;i++){
    html+="DI"+String(i+1)+":"+String(systemStatus.di[i])+" ";
    if((i%8)==7) html+="<br>";
  }
  html+="</p></div><div class='card'><h3>Raw JSON</h3><pre id='jsonbox'></pre>"
        "<script>const data="+j+";document.getElementById('jsonbox').textContent=JSON.stringify(data,null,2);</script>"
        "<p><a href='/'>Back</a> | <a href='/io/diag?raw=1'>Raw JSON</a></p></div></body></html>";
  server.send(200,"text/html", html);
}

// NOWA ZAKŁADKA: Modbus TCP Diagnostics (tekstowa dokumentacja bez zmian funkcjonalnych)
static void handleModbusTCPDiag(){
  if(!requireAuth()) return;
  String html;
  html.reserve(8000);
  html="<!doctype html><html><head><meta charset='utf-8'><title>Modbus TCP Diagnostics</title>"
       "<meta name='viewport' content='width=device-width,initial-scale=1'>"
       "<style>body{font-family:Arial;margin:0;background:#f5f7fb;color:#222}"
       ".wrap{max-width:1100px;margin:0 auto;padding:18px}"
       ".card{background:#fff;padding:16px;border-radius:12px;box-shadow:0 2px 6px rgba(0,0,0,.08);margin-bottom:14px}"
       "table{width:100%;border-collapse:collapse}th,td{padding:8px 10px;border-bottom:1px solid #e3e7ef;text-align:left;font-size:13px}"
       "code,pre{background:#0e1014;color:#e6edf3;padding:10px;border-radius:8px;overflow:auto}"
       ".mono{font-family:monospace}.muted{color:#666;font-size:12px}</style></head><body>"
       "<div class='wrap'><h1>Modbus TCP Diagnostics</h1>"
       "<div class='card'><h3>Konfiguracja</h3><table>"
       "<tr><td>TCP enabled</td><td>"+String(tcpCfg.enabled?"ON":"OFF")+"</td></tr>"
       "<tr><td>TCP port</td><td>"+String(tcpCfg.port)+"</td></tr>"
       "<tr><td>Ethernet IP</td><td>"+ETH.localIP().toString()+"</td></tr>"
       "<tr><td>AP IP</td><td>"+(systemStatus.ap_active?WiFi.softAPIP().toString():"(inactive)")+"</td></tr>"
       "<tr><td>STA IP</td><td>"+(systemStatus.sta_active?WiFi.localIP().toString():"(inactive)")+"</td></tr>"
       "</table><p class='muted'>Serwer ModbusTCP działa w ramach biblioteki emelianov (ModbusIP). Rejestry TCP są współdzielone z RTU (falownik Delta ME300) poprzez mapę HREG/IREG/COIL/ISTS.</p></div>"
       "<div class='card'><h3>Mapa współdzielonych rejestrów (TCP ↔ RTU/ME300)</h3>"
       "<p>Tablica współdzielona odpowiada:</p>"
       "<ul>"
       "<li>HREG (holding, write/read): sterowanie falownikiem i zadane wartości</li>"
       "<li>IREG (input, read-only): telemetria falownika</li>"
       "<li>COIL (cewki, bity): wyjścia przekaźnikowe Y01..Y16 (i E16 jeśli obecne)</li>"
       "<li>ISTS (discrete inputs): wejścia DI01..DI16</li>"
       "</ul>"
       "<h4>Indeksy HREG (sterowanie ME300)</h4>"
       "<table><tr><th>Index (HREG)</th><th>Opis</th><th>RTU reg</th><th>Skala</th></tr>"
       "<tr><td>0 (REG_CONTROL_WORD)</td><td>Słowo sterujące (bitowe): RUN/STOP/JOG/DIR/ACC set/preset</td><td>0x2000</td><td>-</td></tr>"
       "<tr><td>1 (REG_FREQUENCY_SET)</td><td>Częstotliwość zadana (0.01 Hz)</td><td>0x2001</td><td>0.01 Hz</td></tr>"
       "<tr><td>2 (REG_ACCEL_TIME)</td><td>Czas przysp. (opcjonalnie)</td><td>(param.)</td><td>-</td></tr>"
       "<tr><td>3 (REG_DECEL_TIME)</td><td>Czas zwalniania (opcjonalnie)</td><td>(param.)</td><td>-</td></tr>"
       "</table>"
       "<h4>Indeksy IREG (telemetria ME300)</h4>"
       "<table><tr><th>Index (IREG)</th><th>Opis</th><th>RTU reg</th><th>Skala</th></tr>"
       "<tr><td>0 (REG_STATUS_WORD)</td><td>Status podstawowy napędu</td><td>0x2101</td><td>-</td></tr>"
       "<tr><td>1 (REG_OUTPUT_FREQ)</td><td>Częstotliwość wyjściowa</td><td>0x2103</td><td>0.01 Hz</td></tr>"
       "<tr><td>2 (REG_OUTPUT_CURRENT)</td><td>Prąd wyjściowy</td><td>0x2104</td><td>0.01 A / 0.1 A</td></tr>"
       "<tr><td>3 (REG_OUTPUT_VOLTAGE)</td><td>Napięcie wyjściowe</td><td>0x2106</td><td>0.1 V</td></tr>"
       "<tr><td>4 (REG_OUTPUT_POWER)</td><td>Moc wyjściowa</td><td>0x2105</td><td>(wg ME300)</td></tr>"
       "<tr><td>5 (REG_DC_BUS_VOLTAGE)</td><td>Napięcie szyny DC</td><td>0x2105</td><td>0.1 V</td></tr>"
       "<tr><td>6 (REG_RPM)</td><td>Obroty/min (jeśli dostępne)</td><td>0x2104</td><td>-</td></tr>"
       "<tr><td>7 (REG_FAULT_CODE)</td><td>Kod ostrzeżeń/błędów</td><td>0x2100</td><td>-</td></tr>"
       "</table>"
       "<p class='muted'>Dokładne odwzorowanie adresów RTU (Modbus address 0x2000..0x2106) i ich skali zgodnie z załącznikiem rejestry_ME300_Version2.csv.</p>"
       "</div>"
       "<div class='card'><h3>Komunikacja i składnia poleceń (TCP klient)</h3>"
       "<p>Operacje poprzez Modbus TCP (klient):</p>"
       "<ul>"
       "<li>Write Single Holding Register (FC=06) – zapis do HREG indeksów (np. REG_CONTROL_WORD, REG_FREQUENCY_SET)</li>"
       "<li>Read Holding Registers (FC=03) – odczyt HREG</li>"
       "<li>Read Input Registers (FC=04) – odczyt IREG</li>"
       "<li>Write Single Coil (FC=05) – ustawienie wyjść Y (COIL)</li>"
       "<li>Read Coils (FC=01) / Read Discrete Inputs (FC=02) – odczyt COIL / ISTS</li>"
       "</ul>"
       "<h4>Przykłady (tekstowe)</h4>"
       "<pre>1) Start/Stop/Reset (REG_CONTROL_WORD @ HREG[0])\n"
       " - RUN:  zapisz HREG[0] = 0x0002  (bits 1..0 = 10)\n"
       " - STOP: zapisz HREG[0] = 0x0001  (bits 1..0 = 01)\n"
       " - JOG+RUN: zapisz HREG[0] = 0x0003 (bits 1..0 = 11)\n"
       " - Kierunek: bity 5..4 (01:FWD, 10:REV, 11:CHG)\n"
       " - ACC/DEC set: bity 7..6 (00..11)\n"
       " - Preset freq: bity 11..8 (0000..1111) + bit 12=1 aby aktywować\n"
       "Uwaga: Niektóre bity są tylko do odczytu (bit-dependent), stosuj właściwe kombinacje.</pre>"
       "<pre>2) Ustawienie częstotliwości (REG_FREQUENCY_SET @ HREG[1])\n"
       " - zapis wartości w skali 0.01 Hz, np. 50.00 Hz => 5000</pre>"
       "<pre>3) Odczyt telemetrii (IREG[0..7])\n"
       " - FC04: IREG[1] => Output Frequency (0.01 Hz)\n"
       " - FC04: IREG[2] => Output Current (0.01 A / 0.1 A)\n"
       " - FC04: IREG[3] => Output Voltage (0.1 V)\n"
       " - FC04: IREG[5] => DC Bus Voltage (0.1 V)\n"
       " - FC04: IREG[0] => Status Word (bity wg ME300 dokumentacji)\n"
       "</pre>"
       "<pre>4) Wyjścia przekaźnikowe (COIL[0..15] => Y01..Y16)\n"
       " - FC05: COIL[n] = 1 => ON (przekaźnik LOW active), COIL[n] = 0 => OFF\n"
       " - Odczyt FC01: Coils\n"
       "</pre>"
       "<pre>5) Wejścia dyskretne (ISTS[0..15] => DI01..DI16)\n"
       " - FC02: ISTS[n] – 1 gdy aktywne (wejścia aktywne niskim, dekodowane w aplikacji)\n"
       "</pre>"
       "<p>Mapowanie jest stałe i wspólne – RTU MASTER aktualizuje IREG (telemetria) oraz obsługuje HREG (sterowanie) zgodnie z mapą ME300.</p>"
       "<p class='muted'>Dla pełnej listy bitów Status/Control zobacz rejestry_ME300_Version2.csv (załączone w projekcie).</p>"
       "<p><a href='/'>Back</a></p>"
       "</div></div></body></html>";
  server.send(200,"text/html", html);
}

// Inverter master proxy
static void handleActiveProxy(){
  if(!requireAuth()) return;
  server.send(200,"application/json","{\"info\":\"Use /inverter_master/active (append module)\"}");
}

// Web setup
static void setupWeb(){
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/critical", handleCritical);

  server.on("/mqtt/repub", HTTP_GET, handleMqttTopics);
  server.on("/mqtt/repub/ui", HTTP_GET, handleMqttRepubUI);
  server.on("/mqtt/repub/publish", HTTP_GET, handleMqttRepubPublish);
  server.on("/mqtt/repub/set", HTTP_POST, handleMqttRepubSetPublish);

  server.on("/config", HTTP_GET, handleConfigGet);
  server.on("/config", HTTP_POST, handleConfigPost);

  server.on("/io", HTTP_GET, handleIOPage);
  server.on("/io/state", HTTP_GET, handleIOState);
  server.on("/io/set", HTTP_GET, handleIOSet);

  server.on("/io/diag", HTTP_GET, handleIODiag);

  // NOWY ENDPOINT – Modbus TCP Diagnostics (tylko treść, brak zmian runtime)
  server.on("/modbus/tcp/diag", HTTP_GET, handleModbusTCPDiag);

  server.on("/inverter_master/active", HTTP_GET, handleActiveProxy);

  server.begin();
  Serial.println("[HTTP] Server started");
}

// Tasks
static void taskNet(void*){
  for(;;){
    server.handleClient();
    ensureMqtt();
    mqtt.loop();

    if(wifi_mode_pending_apply){
      wifi_mode_pending_apply=false;
      applyWifiMode(wifi_target_ap);
    }

    if(!netCfg.wifi_ap){
      if(!systemStatus.sta_active && sta_connect_start_ms>0){
        uint32_t elapsed = millis()-sta_connect_start_ms;
        if(elapsed > (uint32_t)netCfg.sta_fb_sec*1000UL){
          Serial.printf("[STA Fallback] No IP after %u ms -> AP\n", elapsed);
          netCfg.wifi_ap=true; saveNetCfg();
          applyWifiMode(true);
          sta_connect_start_ms=0;
        }
      }
    }

    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}
static void taskIO(void*){
  for(;;){
    readInputs();
    updateOutputs();
    uint32_t now=millis();
    if(systemStatus.mqtt_connected){
      if(MQTT_PUBLISH_FULL_STATE && (now - systemStatus.last_io_pub > 5000))
        publishIO();
      if(now - systemStatus.last_mb_pub > 5000)
        publishMB();
    }
    vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

// SETUP / LOOP
void setup(){
  Serial.begin(115200);
  Serial.println("[MASTER v21a FULL + STA/AP Dynamic] Boot");
  loadCfg();
  setupETH();
  setupWiFiInitial();
  setupI2C();
  setupMQTT();
  setupMBTCP();
  setupWeb();
  inverter_master_begin();
  xTaskCreatePinnedToCore(taskNet,"NET",8192,nullptr,1,nullptr,0);
  xTaskCreatePinnedToCore(taskIO,"IO", 6144,nullptr,1,nullptr,1);
  Serial.println("[MASTER v21a FULL] Initialization complete");
}
void loop(){
  vTaskDelay(100/portTICK_PERIOD_MS);
}