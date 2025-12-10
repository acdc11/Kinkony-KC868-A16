/* 
  Multi Inverter MASTER APPEND – Version 21a FIX V6c3 (FULL)
  - Single-page WWW (Status & SID, Sterowanie, Rejestry RAW, Konfiguracja, Diagnostyka RS485)
  - Checklist SID (radio)
  - Status SID w tabeli na bazie /rs485 (sid_stats.active)
  - Anti “forever-active”: aktywność tylko z backendu; TTL dezaktywacji przy braku sukcesów odczytu
  - Persistencja RTU (invrtu + kc868cfg + rtu_par_str)
  - Reapply RTU po ~1.5 s
  - Fallback FC04->FC03 dla rejestrów ME300
  - Rate-limit zmian częstotliwości (freqRateLim / min)
  - Auto-uzupełnienie pola Set Freq z ochroną edycji (nie nadpisuje gdy użytkownik wpisuje)
  - Sterowanie kierunkiem: FWD / REV / CHANGE (dir)
  - Endpointy: active, regs, cmd, config, rs485, rtu_diag, wifi
*/

#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ModbusMaster.h>
#include <ModbusIP_ESP8266.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ETH.h>

#ifdef ESP_IDF_VERSION
#include <nvs_flash.h>
#include <nvs.h>
#include "include/me300_regs.h"
#include "include/web_ui.h"
#endif

#ifndef RS485_RX_PIN
#define RS485_RX_PIN 16
#endif
#ifndef RS485_TX_PIN
#define RS485_TX_PIN 13
#endif

extern WebServer  server;
extern const char* www_realm;
extern ModbusIP   mbTCP;
extern PubSubClient mqtt;

class AutoMultiInverter {
public:
  static constexpr uint8_t  MAX_INV=6;
  static constexpr uint8_t  SID_CANDIDATES[MAX_INV]={1,2,3,4,5,6};

  static constexpr uint8_t  MISS_THRESHOLD=5;
  static constexpr uint32_t START_REPROBE_MS=5000;
  static constexpr uint32_t MAX_REPROBE_MS  =60000;
  static constexpr uint32_t BOOTSTRAP_MS    =10000;

  static constexpr uint16_t REG_CTRL = ME300::REG_CONTROL_WORD;
  static constexpr uint16_t REG_SETF = ME300::REG_SET_FREQUENCY_CONTROL;
  static constexpr uint16_t REG_FLAGS= ME300::REG_CONTROL_FAULT_FLAGS;
  static constexpr uint16_t REG_WARN = ME300::REG_WARNING_ALARM_CODE;
  static constexpr uint16_t REG_DSTAT= ME300::REG_DRIVE_STATUS_BASIC;
  static constexpr uint16_t REG_SETR = ME300::REG_SET_FREQUENCY_REPORTED;
  static constexpr uint16_t REG_FOUT = ME300::REG_OUTPUT_FREQUENCY;
  static constexpr uint16_t REG_IOUT = ME300::REG_OUTPUT_CURRENT;
  static constexpr uint16_t REG_VDC  = ME300::REG_DC_BUS_VOLTAGE;
  static constexpr uint16_t REG_VOUT = ME300::REG_OUTPUT_VOLTAGE;

  struct RTUConfig { uint32_t baud=9600; uint8_t parity=0; uint16_t pollMs=500; };
  struct QueueItem { uint8_t sid; uint16_t reg; uint16_t val; };
  struct FEvent { uint32_t ts; uint16_t raw; };
  struct FHist { FEvent ring[10]; uint8_t head=0; uint32_t recent[8]={0}; };

  void begin();
  void page();
  void apiActive();
  void apiActiveRaw();
  void apiRegs();
  void apiCmd();
  void apiCfgGet();
  void apiCfgPost();
  void apiFreqHist();
  void apiRcStats();
  void apiRs485();
  void apiRTUDiag();
  void apiWifiGet();
  void apiWifiPost();
  void sse();
  String snapshotJson(uint8_t sid);

  bool applyRTU(uint32_t baud,uint8_t parity,uint16_t pollMs);
  void persistRTU(const char* reason);
  void reapplyRTULater();
  void reapplyRTU();
  uint8_t normalizeParity(uint8_t raw);
  uint8_t rtuParityToSerial(uint8_t p);
  void checkNVSStats();
  void clearRTUKeys();

  uint32_t getLastStatePub(){ return lastStatePub[0]; }
  uint32_t getLastDecodePub(){ return lastDecodePub[0]; }

private:
  void loadRTU();
  void loadRTUFromGlobal();
  void saveRTU();
  void mirrorRTUToGlobal();
  void loadMqtt();
  void saveMqtt();
  void loadHistAll();
  void saveHist(uint8_t sid);

  bool rtuReadAuto(uint8_t sid,uint16_t reg,uint16_t& val);
  bool rtuWrite(uint8_t sid,uint16_t reg,uint16_t val);
  void notifyFallbackOnce(uint8_t sid,uint16_t reg,bool ok,uint8_t rc04,uint8_t rc03);
  void processReadResult(uint8_t sid,bool success);
  bool anyActive();
  void attemptInitialSetFreq(uint8_t sid);

  bool enq(uint8_t sid,uint16_t reg,uint16_t val);
  bool deq(QueueItem& it);
  uint8_t queueLen();

  static void taskPollThunk(void* p);
  static void taskWriteThunk(void* p);
  void taskPoll();
  void taskWrites();

  int indexOfSid(uint8_t sid);
  uint16_t baseOf(uint8_t sid);
  void setSlave(uint8_t sid);
  bool freqAllowed(uint8_t sid);
  void recordFreq(uint8_t sid,uint16_t raw);
  void publishState(uint8_t sid);
  void publishDecode(uint8_t sid);
  String parseWarn(uint16_t v);
  String parseDstat(uint16_t v);
  String parseCtrl(uint16_t v);
  String parseFlags(uint16_t v);
  bool auth();

  void activateSid(uint8_t sid, const char* reason);

  Preferences pRTU, pMqtt, pHist, pGlobal;
  RTUConfig rtu;
  ModbusMaster me300;
  String topicBase="KINCONY/INVERTER";
  uint32_t pubPeriod=5000;
  uint8_t  freqRateLim=6;

  bool     active[MAX_INV]={false};
  uint8_t  missCount[MAX_INV]={0};
  uint32_t backoffMs[MAX_INV]={START_REPROBE_MS,START_REPROBE_MS,START_REPROBE_MS,START_REPROBE_MS,START_REPROBE_MS,START_REPROBE_MS};
  uint32_t nextProbeAt[MAX_INV]={0,0,0,0,0,0};
  uint8_t  rrScanIndex=0;

  bool initializedSetFreq[MAX_INV]={false};

  QueueItem q[32]; volatile uint8_t qH=0,qT=0;
  portMUX_TYPE qMux=portMUX_INITIALIZER_UNLOCKED;
  uint32_t rcCount[256]={0};

  uint32_t readOk[MAX_INV]={0}, readFail[MAX_INV]={0};
  uint32_t writeOk[MAX_INV]={0}, writeFail[MAX_INV]={0};
  uint32_t lastOkMs[MAX_INV]={0}, lastFailMs[MAX_INV]={0};

  FHist fh[MAX_INV];
  uint32_t lastStatePub[MAX_INV]={0};
  uint32_t lastDecodePub[MAX_INV]={0};

  uint16_t lastCtrl[MAX_INV]={0xFFFF};
  uint16_t lastSetF[MAX_INV]={0xFFFF};
  uint16_t lastFlags[MAX_INV]={0xFFFF};

  bool fallbackNotified[MAX_INV][7]={{false}};
  uint32_t _reapplyStartTs=0;

  static const char PAGE_HTML[] PROGMEM;
};

// ---- IMPLEMENTACJA ----

void AutoMultiInverter::begin(){
  loadRTU();
  loadRTUFromGlobal();
  loadMqtt();
  loadHistAll();

  Serial.printf("[AutoMulti] RTU after load: baud=%lu par=%u poll=%u\n",
      (unsigned long)rtu.baud,(unsigned)rtu.parity,(unsigned)rtu.pollMs);

  Serial2.begin(rtu.baud, rtuParityToSerial(rtu.parity), RS485_RX_PIN, RS485_TX_PIN);
  me300.begin(SID_CANDIDATES[0], Serial2);

  server.on("/inverter_master",               HTTP_GET, std::bind(&AutoMultiInverter::page,this));
  server.on("/inverter_master/active",        HTTP_GET, std::bind(&AutoMultiInverter::apiActive,this));
  server.on("/inverter_master/active_raw",    HTTP_GET, std::bind(&AutoMultiInverter::apiActiveRaw,this));
  server.on("/inverter_master/regs",          HTTP_GET, std::bind(&AutoMultiInverter::apiRegs,this));
  server.on("/inverter_master/cmd",           HTTP_GET, std::bind(&AutoMultiInverter::apiCmd,this));
  server.on("/inverter_master/config",        HTTP_GET, std::bind(&AutoMultiInverter::apiCfgGet,this));
  server.on("/inverter_master/config",        HTTP_POST,std::bind(&AutoMultiInverter::apiCfgPost,this));
  server.on("/inverter_master/freq_history",  HTTP_GET, std::bind(&AutoMultiInverter::apiFreqHist,this));
  server.on("/inverter_master/rc_stats",      HTTP_GET, std::bind(&AutoMultiInverter::apiRcStats,this));
  server.on("/inverter_master/events",        HTTP_GET, std::bind(&AutoMultiInverter::sse,this));
  server.on("/inverter_master/rs485",         HTTP_GET, std::bind(&AutoMultiInverter::apiRs485,this));
  server.on("/inverter_master/rtu_diag",      HTTP_GET, std::bind(&AutoMultiInverter::apiRTUDiag,this));
  server.on("/inverter_master/wifi",          HTTP_GET, std::bind(&AutoMultiInverter::apiWifiGet,this));
  server.on("/inverter_master/wifi",          HTTP_POST,std::bind(&AutoMultiInverter::apiWifiPost,this));

  xTaskCreatePinnedToCore(taskPollThunk,"IM_AutoPoll",8192,this,1,nullptr,1);
  xTaskCreatePinnedToCore(taskWriteThunk,"IM_AutoWrite",4096,this,1,nullptr,1);

  _reapplyStartTs=millis();

  Serial.printf("[AutoMulti v21a FIX V6c3 FULL SinglePage=ON] SIDs: ");
  for(int i=0;i<MAX_INV;i++){ Serial.print(SID_CANDIDATES[i]); if(i<MAX_INV-1) Serial.print(","); }
  Serial.printf(" | baud=%lu par=%u poll=%u freq_rate=%u\n",
      (unsigned long)rtu.baud,(unsigned)rtu.parity,(unsigned)rtu.pollMs,(unsigned)freqRateLim);
}

bool AutoMultiInverter::applyRTU(uint32_t baud,uint8_t parity,uint16_t pollMs){
  rtu.baud=baud;
  rtu.parity=normalizeParity(parity);
  rtu.pollMs=constrain((int)pollMs,100,5000);
  Serial.printf("[RTUCFG] applyRTU baud=%lu par=%u poll=%u\n",(unsigned long)rtu.baud,rtu.parity,rtu.pollMs);
  Serial2.begin(rtu.baud, rtuParityToSerial(rtu.parity), RS485_RX_PIN, RS485_TX_PIN);
  return true;
}

// ---------- NVS / LOAD SAVE ----------
void AutoMultiInverter::loadRTU(){
  pRTU.begin("invrtu", true);
  uint32_t b = pRTU.getULong("baud",9600);
  uint8_t  p = pRTU.getUChar("par",0);
  uint16_t pl= pRTU.getUShort("poll",500);
  pRTU.end();
  rtu.baud  = (b==0)?9600:b;
  rtu.parity= normalizeParity(p);
  rtu.pollMs= constrain((int)pl,100,5000);
  Serial.printf("[RTUCFG] load invrtu baud=%lu par=%u poll=%u\n",(unsigned long)rtu.baud,rtu.parity,rtu.pollMs);
}
void AutoMultiInverter::loadRTUFromGlobal(){
  pGlobal.begin("kc868cfg", true);
  uint32_t b   = pGlobal.getULong ("rtu_baud", rtu.baud);
  uint8_t  p   = pGlobal.getUChar ("rtu_par",  rtu.parity);
  String   ps  = pGlobal.getString("rtu_par_str", "");
  uint16_t pl  = pGlobal.getUShort("rtu_poll", rtu.pollMs);
  pGlobal.end();
  uint8_t np=normalizeParity(p);
  if(ps.length()){
    String s=ps; s.toUpperCase();
    if(s=="8E1") np=1; else if(s=="8O1") np=2; else np=0;
  }
  bool diff = (b!=rtu.baud) || (np!=rtu.parity) || (pl!=rtu.pollMs);
  if(diff){
    Serial.printf("[RTUCFG] override from kc868cfg baud=%lu par=%u poll=%u\n",(unsigned long)b,np,pl);
    rtu.baud=b; rtu.parity=np; rtu.pollMs=constrain((int)pl,100,5000);
  } else {
    Serial.println("[RTUCFG] kc868cfg matches invrtu (no override)");
  }
}
void AutoMultiInverter::saveRTU(){
  pRTU.begin("invrtu", false);
  pRTU.putULong("baud", rtu.baud);
  pRTU.putUChar("par",  rtu.parity);
  pRTU.putUShort("poll",rtu.pollMs);
  pRTU.end();
  Serial.printf("[RTUCFG] saved invrtu baud=%lu par=%u poll=%u\n",(unsigned long)rtu.baud,rtu.parity,rtu.pollMs);
}
void AutoMultiInverter::mirrorRTUToGlobal(){
  pGlobal.begin("kc868cfg", false);
  pGlobal.putULong ("rtu_baud", rtu.baud);
  pGlobal.putUChar ("rtu_par",  rtu.parity);
  pGlobal.putUShort("rtu_poll", rtu.pollMs);
  const char* ps=(rtu.parity==1)?"8E1":(rtu.parity==2)?"8O1":"8N1";
  pGlobal.putString("rtu_par_str", ps);
  pGlobal.end();
  Serial.printf("[RTUCFG] saved kc868cfg baud=%lu par=%u poll=%u (par_str=%s)\n",
      (unsigned long)rtu.baud,rtu.parity,rtu.pollMs,ps);
}
void AutoMultiInverter::loadMqtt(){
  pMqtt.begin("invmqtt", true);
  topicBase   = pMqtt.getString("topic","KINCONY/INVERTER");
  pubPeriod   = pMqtt.getUInt("period",5000);
  freqRateLim = pMqtt.getUChar("freq_rate",6);
  pMqtt.end();
  if(topicBase.length()==0) topicBase="KINCONY/INVERTER";
  pubPeriod=constrain((int)pubPeriod,1000,3600000);
  if(freqRateLim>60) freqRateLim=60;
}
void AutoMultiInverter::saveMqtt(){
  pMqtt.begin("invmqtt", false);
  pMqtt.putString("topic",topicBase);
  pMqtt.putUInt("period",pubPeriod);
  pMqtt.putUChar("freq_rate",freqRateLim);
  pMqtt.end();
}
void AutoMultiInverter::loadHistAll(){
  pHist.begin("invhist", true);
  for(int i=0;i<MAX_INV;i++){
    String key="fh"+String(SID_CANDIDATES[i]);
    String blob=pHist.getString(key.c_str(), "");
    fh[i].head=0;
    for(int j=0;j<10;j++){ fh[i].ring[j].ts=0; fh[i].ring[j].raw=0; }
    int pos=0;
    while(pos<blob.length()){
      int nl=blob.indexOf('\n',pos); if(nl<0) nl=blob.length();
      String ln=blob.substring(pos,nl); ln.trim();
      if(ln.length()){
        int c=ln.indexOf(','); if(c>0){
          fh[i].ring[fh[i].head].ts=ln.substring(0,c).toInt();
          fh[i].ring[fh[i].head].raw=ln.substring(c+1).toInt();
          fh[i].head=(fh[i].head+1)%10;
        }
      }
      pos=nl+1;
    }
  }
  pHist.end();
}
void AutoMultiInverter::saveHist(uint8_t sid){
  int idx=indexOfSid(sid); if(idx<0) return;
  String out;
  for(int i=0;i<10;i++){
    int k=(fh[idx].head+i)%10;
    if(!fh[idx].ring[k].ts) continue;
    out += String(fh[idx].ring[k].ts)+","+String(fh[idx].ring[k].raw)+"\n";
  }
  pHist.begin("invhist", false);
  String key="fh"+String(sid);
  pHist.putString(key.c_str(), out);
  pHist.end();
}

// ---- Helpers indexes ----
int AutoMultiInverter::indexOfSid(uint8_t sid){ for(int i=0;i<MAX_INV;i++) if(SID_CANDIDATES[i]==sid) return i; return -1; }
uint16_t AutoMultiInverter::baseOf(uint8_t sid){ int idx=indexOfSid(sid); if(idx<0) return 0xFFFF; return idx*16; }

// ---- Queue ----
bool AutoMultiInverter::enq(uint8_t sid,uint16_t reg,uint16_t val){
  portENTER_CRITICAL(&qMux);
  bool full=((qH+1)%32)==qT;
  if(full){ rcCount[0xFE]++; portEXIT_CRITICAL(&qMux); return false; }
  q[qH]={sid,reg,val}; qH=(qH+1)%32;
  portEXIT_CRITICAL(&qMux);
  return true;
}
bool AutoMultiInverter::deq(QueueItem& it){
  portENTER_CRITICAL(&qMux);
  bool empty=(qH==qT);
  if(empty){ portEXIT_CRITICAL(&qMux); return false; }
  it=q[qT]; qT=(qT+1)%32;
  portEXIT_CRITICAL(&qMux);
  return true;
}
uint8_t AutoMultiInverter::queueLen(){
  portENTER_CRITICAL(&qMux);
  uint8_t len=(qH>=qT)?(qH-qT):(uint8_t)(32-(qT-qH));
  portEXIT_CRITICAL(&qMux);
  return len;
}

// ---- RTU ops ----
void AutoMultiInverter::setSlave(uint8_t sid){
#if defined(MODBUSMASTER_VERSION)
  me300.setSlave(sid);
#else
  me300.begin(sid,Serial2);
#endif
}
bool AutoMultiInverter::rtuReadAuto(uint8_t sid,uint16_t reg,uint16_t& val){
  int idx=indexOfSid(sid); if(idx<0) return false;
  setSlave(sid);
  uint8_t rc=me300.readInputRegisters(reg,1);
  rcCount[rc]++;
  if(rc==me300.ku8MBSuccess){
    val=me300.getResponseBuffer(0);
    readOk[idx]++; lastOkMs[idx]=millis();
    return true;
  }
  uint8_t rc2=me300.readHoldingRegisters(reg,1);
  rcCount[rc2]++;
  if(rc2==me300.ku8MBSuccess){
    val=me300.getResponseBuffer(0);
    notifyFallbackOnce(sid,reg,true,rc,rc2);
    readOk[idx]++; lastOkMs[idx]=millis();
    return true;
  }
  notifyFallbackOnce(sid,reg,false,rc,rc2);
  readFail[idx]++; lastFailMs[idx]=millis();
  return false;
}
bool AutoMultiInverter::rtuWrite(uint8_t sid,uint16_t reg,uint16_t val){
  int idx=indexOfSid(sid); if(idx<0) return false;
  setSlave(sid);
  uint8_t rc=me300.writeSingleRegister(reg,val);
  rcCount[rc]++;
  if(rc!=me300.ku8MBSuccess){
    writeFail[idx]++; lastFailMs[idx]=millis();
    return false;
  }
  writeOk[idx]++; lastOkMs[idx]=millis();
  return true;
}
void AutoMultiInverter::notifyFallbackOnce(uint8_t sid,uint16_t reg,bool ok,uint8_t rc04,uint8_t rc03){
  int m=indexOfSid(sid); if(m<0) return;
  int idx=-1;
  switch(reg){
    case REG_WARN: idx=0;break;
    case REG_DSTAT:idx=1;break;
    case REG_SETR: idx=2;break;
    case REG_FOUT: idx=3;break;
    case REG_IOUT: idx=4;break;
    case REG_VDC:  idx=5;break;
    case REG_VOUT: idx=6;break;
    default: return;
  }
  if(!fallbackNotified[m][idx]){
    if(ok) Serial.printf("[AutoMulti] Fallback FC04->FC03 OK SID=%u reg=0x%04X\n",sid,reg);
    else   Serial.printf("[AutoMulti] Read fail SID=%u reg=0x%04X rc04=%02X rc03=%02X\n",sid,reg,rc04,rc03);
    fallbackNotified[m][idx]=true;
  }
}
void AutoMultiInverter::processReadResult(uint8_t sid,bool success){
  int idx=indexOfSid(sid); if(idx<0) return;
  if(success){
    if(!active[idx]){
      active[idx]=true;
      missCount[idx]=0;
      backoffMs[idx]=START_REPROBE_MS;
      nextProbeAt[idx]=0;
      initializedSetFreq[idx]=false;
      Serial.printf("[AutoMulti] SID %u ACTIVATED\n", sid);
    } else missCount[idx]=0;
  } else {
    if(active[idx]){
      missCount[idx]++;
      if(missCount[idx]>=MISS_THRESHOLD){
        active[idx]=false;
        missCount[idx]=0;
        backoffMs[idx]=START_REPROBE_MS;
        nextProbeAt[idx]=millis()+backoffMs[idx];
        Serial.printf("[AutoMulti] SID %u INACTIVE\n", sid);
      }
    }
  }
}
bool AutoMultiInverter::anyActive(){ for(int i=0;i<MAX_INV;i++) if(active[i]) return true; return false; }

void AutoMultiInverter::attemptInitialSetFreq(uint8_t sid){
  int idx=indexOfSid(sid); if(idx<0) return;
  if(!active[idx]) return;
  if(initializedSetFreq[idx]) return;
  uint16_t base=baseOf(sid);
  uint16_t reported=mbTCP.Ireg(base+2);
  uint16_t currentSet=mbTCP.Hreg(base+1);
  if(reported>0 && (currentSet==0 || lastSetF[idx]==0xFFFF)){
    mbTCP.Hreg(base+1, reported);
    lastSetF[idx]=reported;
    initializedSetFreq[idx]=true;
    Serial.printf("[AutoMulti] Init set_freq from reported (SID=%u raw=%u)\n", sid, reported);
  } else {
    initializedSetFreq[idx]=true;
    Serial.printf("[AutoMulti] Skip init set_freq (SID=%u reported=%u current=%u)\n", sid, reported, currentSet);
  }
}

void AutoMultiInverter::activateSid(uint8_t sid, const char* reason){
  int idx=indexOfSid(sid); if(idx<0) return;
  if(active[idx]) return;
  active[idx]=true;
  missCount[idx]=0;
  backoffMs[idx]=START_REPROBE_MS;
  nextProbeAt[idx]=0;
  initializedSetFreq[idx]=false;
  Serial.printf("[AutoMulti] SID %u ACTIVATED (%s)\n", sid, reason?reason:"auto");
}

// ---- Tasks ----
void AutoMultiInverter::taskPollThunk(void* p){ ((AutoMultiInverter*)p)->taskPoll(); }
void AutoMultiInverter::taskWriteThunk(void* p){ ((AutoMultiInverter*)p)->taskWrites(); }

void AutoMultiInverter::taskPoll(){
  bool bootstrap=true;
  uint32_t bootstrapStart=millis();
  for(;;){
    uint32_t loopStart=millis();
    if(bootstrap){
      for(int i=0;i<MAX_INV;i++){
        uint8_t sid=SID_CANDIDATES[i];
        uint16_t v;
        bool ok=rtuReadAuto(sid,REG_WARN,v);
        if(ok) mbTCP.Ireg(baseOf(sid)+0,v);
        processReadResult(sid, ok);
      }
      if(anyActive() || millis()-bootstrapStart>BOOTSTRAP_MS) bootstrap=false;
    } else {
      if(anyActive()){
        for(int i=0;i<MAX_INV;i++){
          if(!active[i]) continue;
          uint8_t sid=SID_CANDIDATES[i];
          uint16_t base=baseOf(sid);
          uint16_t v;
          int idx = indexOfSid(sid);
          bool okWarn=false;
          if(rtuReadAuto(sid,REG_WARN,v)){ mbTCP.Ireg(base+0,v); okWarn=true; }
          else processReadResult(sid,false);

          if(rtuReadAuto(sid,REG_DSTAT,v)){ mbTCP.Ireg(base+1,v); if(!active[idx]) activateSid(sid,"alt-reg ME300::REG_DRIVE_STATUS_BASIC"); }
          if(rtuReadAuto(sid,REG_SETR,v)) { mbTCP.Ireg(base+2,v); if(!active[idx]) activateSid(sid,"alt-reg ME300::REG_SET_FREQUENCY_REPORTED"); }
          if(rtuReadAuto(sid,REG_FOUT,v)) { mbTCP.Ireg(base+3,v); if(!active[idx]) activateSid(sid,"alt-reg ME300::REG_OUTPUT_FREQUENCY"); }
          if(rtuReadAuto(sid,REG_IOUT,v)) { mbTCP.Ireg(base+4,v); if(!active[idx]) activateSid(sid,"alt-reg ME300::REG_OUTPUT_CURRENT"); }
          if(rtuReadAuto(sid,REG_VDC,v))  { mbTCP.Ireg(base+5,v); if(!active[idx]) activateSid(sid,"alt-reg ME300::REG_DC_BUS_VOLTAGE"); }
          if(rtuReadAuto(sid,REG_VOUT,v)) { mbTCP.Ireg(base+6,v); if(!active[idx]) activateSid(sid,"alt-reg ME300::REG_OUTPUT_VOLTAGE"); }

          if(okWarn) processReadResult(sid,true);

          // TTL dezaktywacji
          uint32_t now=millis();
          uint32_t ttl = max<uint32_t>((uint32_t)rtu.pollMs * 8, (uint32_t)5000);
          if(active[idx] && (now - lastOkMs[idx] > ttl)){
            active[idx]=false;
            missCount[idx]=0;
            backoffMs[idx]=START_REPROBE_MS;
            nextProbeAt[idx]=now+backoffMs[idx];
            Serial.printf("[AutoMulti] SID %u DEACTIVATED (TTL)\n", sid);
          }

          attemptInitialSetFreq(sid);

          if(mqtt.connected()){
            if(now - lastStatePub[i]  >= pubPeriod) publishState(sid);
            if(now - lastDecodePub[i] >= pubPeriod) publishDecode(sid);
          }
        }
        uint32_t now=millis();
        for(int i=0;i<MAX_INV;i++){
          if(active[i]) continue;
          if(nextProbeAt[i]==0 || now>=nextProbeAt[i]){
            uint8_t sid=SID_CANDIDATES[i];
            uint16_t dummy;
            bool ok=rtuReadAuto(sid,REG_WARN,dummy);
            processReadResult(sid, ok);
            if(!active[i]){
              backoffMs[i]=min<uint32_t>(backoffMs[i]*2,MAX_REPROBE_MS);
              nextProbeAt[i]=now+backoffMs[i];
            } else {
              backoffMs[i]=START_REPROBE_MS;
              nextProbeAt[i]=0;
            }
          }
        }
      } else {
        uint8_t i=rrScanIndex%MAX_INV;
        rrScanIndex=(rrScanIndex+1)%MAX_INV;
        uint8_t sid=SID_CANDIDATES[i];
        uint16_t v;
        bool ok=rtuReadAuto(sid,REG_WARN,v);
        if(ok) mbTCP.Ireg(baseOf(sid)+0,v);
        processReadResult(sid, ok);
      }
    }
    uint32_t elapsed=millis()-loopStart;
    int32_t wait=(int32_t)rtu.pollMs - (int32_t)elapsed;
    if(wait<10) wait=10;

    reapplyRTULater();
    vTaskDelay(wait/portTICK_PERIOD_MS);
  }
}

void AutoMultiInverter::taskWrites(){
  for(;;){
    for(int i=0;i<MAX_INV;i++){
      uint8_t sid=SID_CANDIDATES[i];
      if(!active[i]) continue;
      uint16_t base=baseOf(sid);
      uint16_t ctrl=mbTCP.Hreg(base+0);
      uint16_t setf=mbTCP.Hreg(base+1);
      uint16_t flags=mbTCP.Hreg(base+2);

      if(ctrl!=lastCtrl[i]){ enq(sid,REG_CTRL,ctrl); lastCtrl[i]=ctrl; }
      if(setf!=lastSetF[i]){
        if(freqAllowed(sid)){
          recordFreq(sid,setf);
          enq(sid,REG_SETF,setf);
          lastSetF[i]=setf;
        } else {
          mbTCP.Hreg(base+1,lastSetF[i]);
          rcCount[0xF0]++;
          if(mqtt.connected()){
            String j="{\"class\":\"cis\",\"component\":\"inverter\",\"sid\":"+String(sid)+",\"severity\":\"info\",\"message\":\"frequency change rate-limited\"}";
            mqtt.publish("KINCONY/ALERT", j.c_str(), false);
          }
        }
      }
      if(flags!=lastFlags[i]){
        enq(sid,REG_FLAGS,flags);
        if(flags & 0x0002){
          uint16_t nf=flags & ~0x0002;
          mbTCP.Hreg(base+2,nf);
          lastFlags[i]=nf;
        } else lastFlags[i]=flags;
      }
    }
    QueueItem it;
    if(deq(it)) rtuWrite(it.sid,it.reg,it.val);
    reapplyRTULater();
    vTaskDelay(70/portTICK_PERIOD_MS);
  }
}

// ---- Freq helpers ----
bool AutoMultiInverter::freqAllowed(uint8_t sid){
  if(freqRateLim==0) return true;
  int idx=indexOfSid(sid); if(idx<0) return true;
  uint8_t cnt=0; uint32_t now=millis();
  for(int i=0;i<8;i++) if(fh[idx].recent[i] && now - fh[idx].recent[i] <=60000UL) cnt++;
  return cnt < freqRateLim;
}
void AutoMultiInverter::recordFreq(uint8_t sid,uint16_t raw){
  int idx=indexOfSid(sid); if(idx<0) return;
  fh[idx].ring[fh[idx].head].ts=millis();
  fh[idx].ring[fh[idx].head].raw=raw;
  fh[idx].head=(fh[idx].head+1)%10;
  saveHist(sid);
  for(int i=7;i>0;i--) fh[idx].recent[i]=fh[idx].recent[i-1];
  fh[idx].recent[0]=millis();
}

// ---- Publish ----
void AutoMultiInverter::publishState(uint8_t sid){
  int idx=indexOfSid(sid); if(idx<0 || !active[idx]) return;
  uint16_t base=baseOf(sid);
  String j; j.reserve(256);
  j="{\"sid\":"+String(sid)+",\"warn_fault\":"+String(mbTCP.Ireg(base+0))+
    ",\"drive_status\":"+String(mbTCP.Ireg(base+1))+
    ",\"set_reported\":"+String(mbTCP.Ireg(base+2))+
    ",\"out_freq\":"+String(mbTCP.Ireg(base+3))+
    ",\"out_curr\":"+String(mbTCP.Ireg(base+4))+
    ",\"dc_bus\":"+String(mbTCP.Ireg(base+5))+
    ",\"out_volt\":"+String(mbTCP.Ireg(base+6))+
    ",\"ctrl_word\":"+String(mbTCP.Hreg(base+0))+
    ",\"set_freq\":"+String(mbTCP.Hreg(base+1))+
    ",\"flags\":"+String(mbTCP.Hreg(base+2))+"}";
  mqtt.publish((topicBase+"/"+String(sid)+"/state").c_str(), j.c_str(), false);
  lastStatePub[idx]=millis();
}
void AutoMultiInverter::publishDecode(uint8_t sid){
  int idx=indexOfSid(sid); if(idx<0 || !active[idx]) return;
  uint16_t base=baseOf(sid);
  String d; d.reserve(256);
  d="{\"sid\":"+String(sid)+",\"ctrl\":"+parseCtrl(mbTCP.Hreg(base+0))+
    ",\"flags\":"+parseFlags(mbTCP.Hreg(base+2))+
    ",\"warn_fault\":"+parseWarn(mbTCP.Ireg(base+0))+
    ",\"drive_status\":"+parseDstat(mbTCP.Ireg(base+1))+"}";
  mqtt.publish((topicBase+"/"+String(sid)+"/decode").c_str(), d.c_str(), false);
  lastDecodePub[idx]=millis();
}

// ---- Parsing ----
String AutoMultiInverter::parseWarn(uint16_t v){
  uint8_t fault=v&0xFF, warn=(v>>8)&0xFF;
  return String("{\"raw\":")+v+",\"warning_code\":"+warn+",\"fault_code\":"+fault+"}";
}
String AutoMultiInverter::parseDstat(uint16_t v){
  uint8_t st=v&0x3; const char* s=(st==0)?"Stopped":(st==1)?"Braking":(st==2)?"Standby":"Running";
  uint8_t dir=(v>>3)&0x3; const char* d=(dir==0)?"FWD":(dir==1)?"REV->FWD":(dir==2)?"REV":"FWD->REV";
  bool jog=v&(1<<2), fcom=v&(1<<8), fana=v&(1<<9), ccom=v&(1<<10), lock=v&(1<<11), pnl=v&(1<<12);
  String j="{\"raw\":"+String(v)+",\"state\":\""+s+"\",\"direction\":\""+d+"\",\"jog\":"+(jog?"true":"false")+
    ",\"freq_comm\":"+(fcom?"true":"false")+",\"freq_analog\":"+(fana?"true":"false")+
    ",\"comm_control\":"+(ccom?"true":"false")+",\"param_lock\":"+(lock?"true":"false")+
    ",\"panel_download\":"+(pnl?"true":"false")+"}";
  return j;
}
String AutoMultiInverter::parseCtrl(uint16_t v){
  uint8_t st=v&0x3; const char* s=(st==0)?"none":(st==1)?"stop":(st==2)?"run":"jog_run";
  uint8_t dir=(v>>4)&0x3; const char* d=(dir==0)?"none":(dir==1)?"fwd":(dir==2)?"rev":"change";
  uint8_t acc=(v>>6)&0x3; uint8_t fsel=(v>>8)&0xF; const char* fd=(fsel==0)?"main":"preset";
  bool en=v&(1<<12); uint8_t cm=(v>>13)&0x3; const char* cd=(cm==0)?"none":(cm==1)?"panel":(cm==2)?"param_00_21":"change_ctrl";
  String j="{\"raw\":"+String(v)+",\"state\":\""+s+"\",\"direction\":\""+d+"\",\"accel_set\":"+String(acc)+
    ",\"freq_select\":"+String(fsel)+",\"freq_select_desc\":\""+fd+"\",\"enable_functions\":"+(en?"true":"false")+
    ",\"control_mode\":\""+cd+"\"}";
  return j;
}
String AutoMultiInverter::parseFlags(uint16_t v){
  bool ef=v&0x0001,rst=v&0x0002,bb=v&0x0004;
  String j="{\"raw\":"+String(v)+",\"fault_active\":"+(ef?"true":"false")+
    ",\"fault_reset_cmd\":"+(rst?"true":"false")+
    ",\"base_block\":"+(bb?"true":"false")+"}";
  return j;
}

// ---- Auth ----
bool AutoMultiInverter::auth(){
  if(!server.authenticate("admin","darol177")){
    server.requestAuthentication(DIGEST_AUTH, www_realm);
    return false;
  }
  return true;
}

// ---- Persistence helpers ----
uint8_t AutoMultiInverter::normalizeParity(uint8_t raw){ return (raw>2)?0:raw; }
uint8_t AutoMultiInverter::rtuParityToSerial(uint8_t p){
  switch(p){
    case 1: return SERIAL_8E1;
    case 2: return SERIAL_8O1;
    default: return SERIAL_8N1;
  }
}
void AutoMultiInverter::persistRTU(const char* reason){
  saveRTU();
  mirrorRTUToGlobal();
  pRTU.begin("invrtu", true);
  uint32_t b1=pRTU.getULong("baud",0); uint8_t p1=pRTU.getUChar("par",255); uint16_t pl1=pRTU.getUShort("poll",0);
  pRTU.end();
  pGlobal.begin("kc868cfg", true);
  uint32_t b2=pGlobal.getULong("rtu_baud",0); uint8_t p2=pGlobal.getUChar("rtu_par",255); uint16_t pl2=pGlobal.getUShort("rtu_poll",0);
  String ps=pGlobal.getString("rtu_par_str","");
  pGlobal.end();
  Serial.printf("[RTUCFG] persist reason=%s invrtu(%lu,%u,%u) kc868cfg(%lu,%u,%u par_str=%s)\n",
    reason,(unsigned long)b1,p1,pl1,(unsigned long)b2,p2,pl2, ps.c_str());
  checkNVSStats();
}
void AutoMultiInverter::reapplyRTULater(){
  if(_reapplyStartTs==0) return;
  if(millis()-_reapplyStartTs>1500){
    reapplyRTU();
    _reapplyStartTs=0;
  }
}
void AutoMultiInverter::reapplyRTU(){
  Serial.printf("[RTUCFG] reapplyRTU baud=%lu par=%u poll=%u\n",(unsigned long)rtu.baud,rtu.parity,rtu.pollMs);
  Serial2.begin(rtu.baud, rtuParityToSerial(rtu.parity), RS485_RX_PIN, RS485_TX_PIN);
}
void AutoMultiInverter::checkNVSStats(){
#ifdef ESP_IDF_VERSION
  nvs_stats_t st;
  if(nvs_get_stats(nullptr,&st)==ESP_OK){
    Serial.printf("[RTUCFG] NVS stats used_entries=%u free_entries=%u namespaces=%u\n",
      st.used_entries, st.free_entries, st.namespace_count);
  }
#endif
}
void AutoMultiInverter::clearRTUKeys(){
  pRTU.begin("invrtu", false);
  pRTU.remove("baud"); pRTU.remove("par"); pRTU.remove("poll");
  pRTU.end();
  pGlobal.begin("kc868cfg", false);
  pGlobal.remove("rtu_baud"); pGlobal.remove("rtu_par"); pGlobal.remove("rtu_poll"); pGlobal.remove("rtu_par_str");
  pGlobal.end();
  Serial.println("[RTUCFG] Cleared RTU keys in both namespaces");
}

// ---- Handlery WWW / API ----
void AutoMultiInverter::page(){ if(!auth()) return; server.send_P(200,"text/html", PAGE_HTML); }
void AutoMultiInverter::sse(){
  if(!auth()) return;
  if(!server.hasArg("sid")){ server.send(400,"text/plain","sid required"); return; }
  uint8_t sid=(uint8_t)server.arg("sid").toInt();
  if(indexOfSid(sid)<0){ server.send(400,"text/plain","invalid sid"); return; }
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Type","text/event-stream");
  server.sendHeader("Cache-Control","no-cache");
  server.sendHeader("Connection","keep-alive");
  server.send(200);
  WiFiClient c=server.client();
  uint32_t last=millis();
  while(c.connected()){
    if(millis()-last>=rtu.pollMs){
      c.print("event: update\ndata: ");
      c.print(snapshotJson(sid));
      c.print("\n\n");
      c.flush();
      last=millis();
    }
    delay(10);
  }
}
String AutoMultiInverter::snapshotJson(uint8_t sid){
  int idx=indexOfSid(sid); if(idx<0) return "{}";
  uint16_t base=baseOf(sid);
  String j; j.reserve(220);
  j="{\"sid\":"+String(sid)+",\"active\":"+(active[idx]?"true":"false")+
    ",\"warn_fault\":"+String(mbTCP.Ireg(base+0))+
    ",\"drive_status\":"+String(mbTCP.Ireg(base+1))+
    ",\"set_reported\":"+String(mbTCP.Ireg(base+2))+
    ",\"out_freq\":"+String(mbTCP.Ireg(base+3))+
    ",\"out_curr\":"+String(mbTCP.Ireg(base+4))+
    ",\"dc_bus\":"+String(mbTCP.Ireg(base+5))+
    ",\"out_volt\":"+String(mbTCP.Ireg(base+6))+
    ",\"ctrl_word\":"+String(mbTCP.Hreg(base+0))+
    ",\"set_freq\":"+String(mbTCP.Hreg(base+1))+
    ",\"flags\":"+String(mbTCP.Hreg(base+2))+"}";
  return j;
}
void AutoMultiInverter::apiActive(){
  if(!auth()) return;
  String actives; actives.reserve(32);
  String inactives; inactives.reserve(32);
  String ro="[", rf="[";
  for(int i=0;i<MAX_INV;i++){
    if(i){ ro+=","; rf+=","; }
    ro+=String(readOk[i]);
    rf+=String(readFail[i]);
    if(active[i]){
      if(actives.length()) actives+=",";
      actives+=String(SID_CANDIDATES[i]);
    } else {
      if(inactives.length()) inactives+=",";
      inactives+=String(SID_CANDIDATES[i]);
    }
  }
  ro+="]"; rf+="]";
  String cands="[";
  for(int i=0;i<MAX_INV;i++){ cands+=String(SID_CANDIDATES[i]); if(i<MAX_INV-1) cands+=","; }
  cands+="]";
  String j="{\"active_sids\":["+actives+"],\"inactive_sids\":["+inactives+"],\"candidates\":"+cands+
           ",\"read_ok\":"+ro+",\"read_fail\":"+rf+"}";
  server.send(200,"application/json", j);
}
void AutoMultiInverter::apiActiveRaw(){ apiActive(); }
void AutoMultiInverter::apiCfgGet(){
  if(!auth()) return;
  String j; j.reserve(200);
  j="{\"rtu\":{\"baud\":"+String(rtu.baud)+",\"par\":"+String(rtu.parity)+",\"poll\":"+String(rtu.pollMs)+"}";
  j+=",\"mqtt\":{\"topic\":\""+topicBase+"\",\"period\":"+String(pubPeriod)+",\"freq_rate\":"+String(freqRateLim)+"}}";
  server.send(200,"application/json", j);
}
void AutoMultiInverter::apiCfgPost(){
  if(!auth()) return;
  bool rtuChanged=false, mqttChanged=false;
  if(server.hasArg("baud")){
    uint32_t b=server.arg("baud").toInt();
    if(b==9600||b==19200||b==38400||b==57600||b==115200){ rtu.baud=b; rtuChanged=true; }
  }
  if(server.hasArg("par")){
    String p=server.arg("par"); p.toLowerCase();
    if(p=="0"||p=="n"||p=="none") rtu.parity=0,rtuChanged=true;
    else if(p=="1"||p=="e"||p=="even") rtu.parity=1,rtuChanged=true;
    else if(p=="2"||p=="o"||p=="odd")  rtu.parity=2,rtuChanged=true;
  }
  if(server.hasArg("poll")){
    int pl=server.arg("poll").toInt(); if(pl>=100 && pl<=5000){ rtu.pollMs=pl; rtuChanged=true; }
  }
  if(server.hasArg("topic")){
    String t=server.arg("topic"); if(t.length()){ topicBase=t; mqttChanged=true; }
  }
  if(server.hasArg("period")){
    uint32_t p=server.arg("period").toInt(); if(p>=1000 && p<=3600000){ pubPeriod=p; mqttChanged=true; }
  }
  if(server.hasArg("freq_rate")){
    int fr=server.arg("freq_rate").toInt(); if(fr<0) fr=0; if(fr>60) fr=60; freqRateLim=(uint8_t)fr; mqttChanged=true;
  }

  applyRTU(rtu.baud,rtu.parity,rtu.pollMs);
  persistRTU(rtuChanged?"update":"force");
  if(mqttChanged) saveMqtt();

  String resp;
  resp.reserve(128);
  resp += "{\"ok\":true,\"rtu_changed\":";
  resp += (rtuChanged ? "true" : "false");
  resp += ",\"mqtt_changed\":";
  resp += (mqttChanged ? "true" : "false");
  resp += ",\"baud\":";
  resp += String(rtu.baud);
  resp += ",\"par\":";
  resp += String(rtu.parity);
  resp += ",\"poll\":";
  resp += String(rtu.pollMs);
  resp += "}";

  server.send(200,"application/json", resp);
}
void AutoMultiInverter::apiRegs(){
  if(!auth()) return;
  if(!server.hasArg("sid")){
    server.send(400,"application/json","{\"error\":\"sid_required\"}");
    return;
  }
  uint8_t sid = (uint8_t)server.arg("sid").toInt();
  int idx = indexOfSid(sid);
  if(idx < 0){
    server.send(400,"application/json","{\"error\":\"invalid_sid\"}");
    return;
  }
  uint16_t base = baseOf(sid);
  String raw = snapshotJson(sid);
  String dec;
  dec.reserve(260);
  dec = "{\"ctrl\":"+parseCtrl(mbTCP.Hreg(base+0))+
        ",\"flags\":"+parseFlags(mbTCP.Hreg(base+2))+
        ",\"warn_fault\":"+parseWarn(mbTCP.Ireg(base+0))+
        ",\"drive_status\":"+parseDstat(mbTCP.Ireg(base+1))+"}";
  String out;
  out.reserve(raw.length() + dec.length() + 32);
  out = "{\"raw\":"+raw+",\"decode\":"+dec+"}";
  server.send(200,"application/json", out);
}
void AutoMultiInverter::apiCmd(){
  if(!auth()) return;
  if(!server.hasArg("sid")){ server.send(400,"application/json","{\"error\":\"sid_required\"}"); return;}
  uint8_t sid=(uint8_t)server.arg("sid").toInt();
  int idx=indexOfSid(sid); if(idx<0){ server.send(400,"application/json","{\"error\":\"invalid_sid\"}"); return; }

  if(!active[idx]){
    server.send(409,"application/json","{\"error\":\"sid_inactive\"}");
    return;
  }
  uint16_t base=baseOf(sid);
  String c=server.arg("c");
  uint16_t ctrl=mbTCP.Hreg(base+0);
  uint16_t flags=mbTCP.Hreg(base+2);
  if(c=="start"||c=="stop"||c=="jog"){
    if(c=="start") ctrl=(ctrl & ~0x0003)|0x0002;
    else if(c=="stop") ctrl=(ctrl & ~0x0003)|0x0001;
    else ctrl=(ctrl & ~0x0003)|0x0003;
    mbTCP.Hreg(base+0,ctrl); server.send(200,"application/json","{\"ok\":true}"); return;
  }
  if(c=="dir"){
    String v=server.arg("v"); v.toLowerCase();
    ctrl &= ~(0x3<<4);
    if(v=="fwd") ctrl|=(0x1<<4);
    else if(v=="rev") ctrl|=(0x2<<4);
    else if(v=="chg") ctrl|=(0x3<<4);
    mbTCP.Hreg(base+0,ctrl); server.send(200,"application/json","{\"ok\":true}"); return;
  }
  if(c=="acc_set"){
    int v=server.arg("v").toInt(); if(v<0||v>3){ server.send(400,"application/json","{\"error\":\"acc_range\"}"); return; }
    ctrl &= ~(0x3<<6); ctrl |= ((v&0x3)<<6);
    mbTCP.Hreg(base+0,ctrl); server.send(200,"application/json","{\"ok\":true}"); return;
  }
  if(c=="preset"){
    int v=server.arg("v").toInt(); if(v<0||v>15){ server.send(400,"application/json","{\"error\":\"preset_range\"}"); return; }
    ctrl &= ~(0xF<<8); ctrl |= ((v&0xF)<<8); ctrl |= (1<<12);
    mbTCP.Hreg(base+0,ctrl); server.send(200,"application/json","{\"ok\":true}"); return;
  }
  if(c=="preset_off"){
    ctrl &= ~(1<<12); mbTCP.Hreg(base+0,ctrl); server.send(200,"application/json","{\"ok\":true}"); return;
  }
  if(c=="base_block_on"||c=="base_block_off"){
    if(c=="base_block_on") flags|=0x0004; else flags &= ~0x0004;
    mbTCP.Hreg(base+2,flags); server.send(200,"application/json","{\"ok\":true}"); return;
  }
  if(c=="reset"){
    flags|=0x0002; mbTCP.Hreg(base+2,flags); server.send(200,"application/json","{\"ok\":true}"); return;
  }
  if(c=="setf"){
    float hz=server.arg("v").toFloat();
    if(!(hz>=0 && hz<=400)){ server.send(400,"application/json","{\"error\":\"freq_range\"}"); return; }
    uint16_t raw=(uint16_t)round(hz*100.0f);
    mbTCP.Hreg(base+1,raw); server.send(200,"application/json","{\"ok\":true}"); return;
  }
  server.send(400,"application/json","{\"error\":\"unknown_cmd\"}");
}
void AutoMultiInverter::apiFreqHist(){
  if(!auth()) return;
  if(!server.hasArg("sid")){ server.send(400,"application/json","{\"error\":\"sid_required\"}"); return; }
  uint8_t sid=(uint8_t)server.arg("sid").toInt();
  int idx=indexOfSid(sid); if(idx<0){ server.send(400,"application/json","{\"error\":\"invalid_sid\"}"); return; }
  String out="["; bool first=true;
  for(int i=0;i<10;i++){
    int k=(fh[idx].head*i)%10; // intentionally ring order for display history
    k=(fh[idx].head+i)%10;
    if(!fh[idx].ring[k].ts) continue;
    if(!first) out+=",";
    first=false;
    out+="{\"ts\":"+String(fh[idx].ring[k].ts)+",\"raw\":"+String(fh[idx].ring[k].raw)+"}";
  }
  out+="]";
  server.send(200,"application/json", out);
}
void AutoMultiInverter::apiRcStats(){
  if(!auth()) return;
  String j="{\"rc_success\":"+String(rcCount[0x00])+
    ",\"rc_timeout\":"+String(rcCount[0xE2])+
    ",\"rc_crc\":"+String(rcCount[0xE3])+
    ",\"queue_full\":"+String(rcCount[0xFE])+
    ",\"rate_limited\":"+String(rcCount[0xF0])+"}";
  server.send(200,"application/json", j);
}
void AutoMultiInverter::apiRs485(){
  if(!auth()) return;
  String j; j.reserve(1400);
  j="{\"rtu\":{\"baud\":"+String(rtu.baud)+",\"par\":"+String(rtu.parity)+",\"poll\":"+String(rtu.pollMs)+"}";
  j+=",\"pins\":{\"rx\":"+String(RS485_RX_PIN)+",\"tx\":"+String(RS485_TX_PIN)+"}";
  j+=",\"queue_len\":"+String(queueLen());
  j+=",\"sid_stats\":[";
  for(int i=0;i<MAX_INV;i++){
    if(i) j+=",";
    uint32_t now=millis();
    uint32_t npa=nextProbeAt[i];
    j+="{\"sid\":"+String(SID_CANDIDATES[i])+
      ",\"active\":"+(active[i]?"true":"false")+
      ",\"miss\":"+String(missCount[i])+
      ",\"read_ok\":"+String(readOk[i])+
      ",\"read_fail\":"+String(readFail[i])+
      ",\"write_ok\":"+String(writeOk[i])+
      ",\"write_fail\":"+String(writeFail[i])+
      ",\"backoff_ms\":"+String(backoffMs[i])+
      ",\"next_probe_ms\":"+String(npa>now? (npa-now):0)+"}";
  }
  j+="],\"rc_counts\":{\"00\":"+String(rcCount[0x00])+",\"E2\":"+String(rcCount[0xE2])+",\"E3\":"+String(rcCount[0xE3])+",\"FE\":"+String(rcCount[0xFE])+",\"F0\":"+String(rcCount[0xF0])+"}}";
  server.send(200,"application/json", j);
}
void AutoMultiInverter::apiRTUDiag(){
  if(!auth()) return;
  if(server.hasArg("clear")) clearRTUKeys();
#ifdef ESP_IDF_VERSION
  if(server.hasArg("erase_nvs")){
    nvs_flash_erase();
    server.send(200,"text/plain","NVS erased. Restarting...");
    delay(300);
    ESP.restart();
    return;
  }
#endif
  if(server.hasArg("reapply")) reapplyRTU();
  pRTU.begin("invrtu", true);
  uint32_t b1=pRTU.getULong("baud",0); uint8_t p1=pRTU.getUChar("par",255); uint16_t pl1=pRTU.getUShort("poll",0);
  pRTU.end();
  pGlobal.begin("kc868cfg", true);
  uint32_t b2=pGlobal.getULong("rtu_baud",0); uint8_t p2=pGlobal.getUChar("rtu_par",255); uint16_t pl2=pGlobal.getUShort("rtu_poll",0);
  String ps=pGlobal.getString("rtu_par_str","");
  bool wifi_ap=pGlobal.getBool("wifi_ap", true);
  String sta_ssid=pGlobal.getString("sta_ssid","");
  pGlobal.end();
#ifdef ESP_IDF_VERSION
  nvs_stats_t st; bool statsOK=(nvs_get_stats(nullptr,&st)==ESP_OK);
#endif
  String j="{\"current\":{\"baud\":"+String(rtu.baud)+",\"par\":"+String(rtu.parity)+",\"poll\":"+String(rtu.pollMs)+"}";
  j+=",\"invrtu\":{\"baud\":"+String(b1)+",\"par\":"+String(p1)+",\"poll\":"+String(pl1)+"}";
  j+=",\"kc868cfg\":{\"baud\":"+String(b2)+",\"par\":"+String(p2)+",\"poll\":"+String(pl2)+",\"par_str\":\""+ps+"\",\"wifi_ap\":"+(wifi_ap?"true":"false")+",\"sta_ssid\":\""+sta_ssid+"\"}";
#ifdef ESP_IDF_VERSION
  if(statsOK){
    j+=",\"nvs_stats\":{\"used\":"+String(st.used_entries)+",\"free\":"+String(st.free_entries)+",\"namespaces\":"+String(st.namespace_count)+"}";
  }
#endif
  j+="}";
  server.send(200,"application/json", j);
}
void AutoMultiInverter::apiWifiGet(){
  if(!auth()) return;
  pGlobal.begin("kc868cfg", true);
  bool wifi_ap=pGlobal.getBool("wifi_ap", true);
  String sta_ssid=pGlobal.getString("sta_ssid","");
  String sta_pass=pGlobal.getString("sta_pass","");
  pGlobal.end();
  String j="{\"wifi_ap\":"+String(wifi_ap?"true":"false")+",\"sta_ssid\":\""+sta_ssid+"\",\"sta_pass\":\""+sta_pass+"\"}";
  server.send(200,"application/json", j);
}
void AutoMultiInverter::apiWifiPost(){
  if(!auth()) return;
  bool wifi_ap=true;
  if(server.hasArg("mode")){
    String m=server.arg("mode"); m.toUpperCase(); wifi_ap=(m!="STA");
  } else if(server.hasArg("wifi_ap")){
    wifi_ap = server.arg("wifi_ap")!="0";
  }
  String ssid=server.arg("sta_ssid");
  String pass=server.arg("sta_pass");
  pGlobal.begin("kc868cfg", false);
  pGlobal.putBool("wifi_ap", wifi_ap);
  if(!wifi_ap){
    if(ssid.length()) pGlobal.putString("sta_ssid", ssid);
    if(pass.length()) pGlobal.putString("sta_pass", pass);
  }
  pGlobal.end();
  String r="{\"ok\":true,\"wifi_ap\":"+String(wifi_ap?"true":"false")+"}";
  if(server.hasArg("apply") && server.arg("apply")=="1"){
    server.send(200,"application/json", r);
    delay(200);
    ESP.restart();
    return;
  }
  server.send(200,"application/json", r);
}

// ---- HTML (Single-page; Status z /rs485; Set Freq guard; Dir buttons) ----
const char AutoMultiInverter::PAGE_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><title>Multi Inverter MASTER – Single Page</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{font-family:Arial;margin:0;background:#f2f5f8;color:#222}
h2{margin:24px 16px 12px;font-size:20px}
.container{max-width:1250px;margin:0 auto;padding:0 12px}
.card{background:#fff;margin:12px 0;padding:16px 18px;border-radius:12px;box-shadow:0 2px 6px rgba(0,0,0,.12)}
table{width:100%;border-collapse:collapse;font-size:13px;margin-top:6px}
th,td{padding:6px 8px;border-bottom:1px solid #ddd;text-align:left}
th{background:#f9fafb}
.sid-table{border:1px solid #ddd;border-radius:10px;overflow:hidden;width:auto}
.sid-table th,.sid-table td{padding:6px 10px}
.sid-state{font-size:11px;padding:2px 6px;border-radius:10px;border:1px solid #888;display:inline-block}
.sid-state.active{background:#c8f7c5;border-color:#27ae60}
.sid-state.inactive{background:#ffd9b3;border-color:#e67e22}
.btn{background:#1976d2;color:#fff;border:none;padding:8px 12px;border-radius:6px;cursor:pointer;font-size:14px}
.btn[disabled]{opacity:.45;cursor:not-allowed}
.btn-alt{background:#455a64}
.btn-danger{background:#e53935}
.btn-ok{background:#43a047}
input[type=number],input[type=text],select{padding:7px 10px;border:1px solid #bbb;border-radius:6px;font-size:14px}
.flex-row{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-top:8px}
.small-note{font-size:12px;color:#555}
.badge{background:#1976d2;color:#fff;padding:2px 6px;border-radius:4px;font-size:11px}
.anchor-bar{position:sticky;top:0;background:#1e2d3a;color:#fff;padding:8px 12px;display:flex;flex-wrap:wrap;gap:12px;font-size:13px;z-index:15}
.anchor-bar a{color:#fff;text-decoration:none;padding:4px 8px;border-radius:6px;background:#2d4a5f}
.anchor-bar a:hover{background:#1976d2}
.hl{font-weight:bold}
</style>
<script>
let currentSid=1;
let activeList=[];
let logicalActiveList=[];
let pollTimer=null;
let rs485Timer=null;

// Guard dla pola Set Freq – nie nadpisujemy, kiedy użytkownik edytuje
let freqEditing=false;
let freqEditTs=0;
function anchorScroll(id){
  const el=document.getElementById(id);
  if(el) el.scrollIntoView({behavior:'smooth',block:'start'});
}
function setControlEnabled(on){
  document.querySelectorAll('.control-section .btn, .control-section input').forEach(e=>{ e.disabled=!on; });
}
async function loadActive(){
  let d=null;
  try{ d=await fetch('/inverter_master/rs485').then(r=>r.json()); }catch(e){ console.error('rs485 fetch failed',e); return; }
  const stats = Array.isArray(d.sid_stats)? d.sid_stats : [];
  const cands = stats.map(s=>s.sid);
  activeList=[]; logicalActiveList=[];
  stats.forEach(s=>{ if(s.active){ activeList.push(s.sid); logicalActiveList.push(s.sid);} });
  if(cands.indexOf(currentSid)<0 && cands.length) currentSid=cands[0];
  renderSidTable(cands, logicalActiveList);
  updateControlState();
}
function renderSidTable(cands, logicAct){
  const host=document.getElementById('sid_table_host');
  let h='<table class="sid-table"><tr><th>Wybór</th><th>SID</th><th>Stan</th></tr>';
  cands.forEach(s=>{
    const isA=logicAct.indexOf(s)>=0;
    const chk=(s===currentSid)?'checked':'';
    h+=`<tr>
      <td><input type="radio" name="sid_choice" value="${s}" ${chk} onclick="chooseSid(${s})"></td>
      <td>SID ${s}</td>
      <td><span class="sid-state ${isA?'active':'inactive'}">${isA?'Active':'Inactive'}</span></td>
    </tr>`;
  });
  h+='</table>';
  host.innerHTML=h;
  document.getElementById('current_sid_regs').textContent=currentSid;
  document.getElementById('current_sid_ctrl').textContent=currentSid;
}
function chooseSid(s){
  currentSid=parseInt(s);
  document.getElementById('current_sid_regs').textContent=currentSid;
  document.getElementById('current_sid_ctrl').textContent=currentSid;
  updateControlState();
  fetchRegs();
}
function updateControlState(){
  const on = logicalActiveList.indexOf(currentSid)>=0;
  setControlEnabled(on);
}
async function fetchRegs(){
  try{
    const r=await fetch('/inverter_master/regs?sid='+currentSid).then(x=>x.json());
    if(!r||!r.raw) return;
    const raw=r.raw;
    set('rw_warn',raw.warn_fault);
    set('rw_dstat',raw.drive_status);
    set('rw_setr',raw.set_reported);
    set('rw_fout',raw.out_freq);
    set('rw_iout',raw.out_curr);
    set('rw_vdc',raw.dc_bus);
    set('rw_vout',raw.out_volt);
    set('rw_ctrl',raw.ctrl_word);
    set('rw_setf',raw.set_freq);
    set('rw_flags',raw.flags);
    set('sc_setr',(raw.set_reported/100).toFixed(2)+' Hz');
    set('sc_fout',(raw.out_freq/100).toFixed(2)+' Hz');
    set('sc_iout',(raw.out_curr/100).toFixed(2)+' A');
    set('sc_vdc',(raw.dc_bus/10).toFixed(1)+' V');
    set('sc_vout',(raw.out_volt/10).toFixed(1)+' V');

    // Auto-uzupełnienie pola Set Freq – tylko gdy NIE edytujesz
    const inf=document.getElementById('in_freq');
    if(inf){
      const now=Date.now();
      const currentlyActive = document.activeElement===inf;
      const editing = freqEditing || currentlyActive || (now - freqEditTs < 1200);
      if(!editing){
        const newV=(raw.set_reported/100).toFixed(2);
        const autoPrev=inf.getAttribute('data-autofill')||'';
        // Nadpisz tylko jeśli pole puste lub nadal jest w trybie auto
        if(inf.value==='' || inf.value===autoPrev){
          inf.value=newV;
          inf.setAttribute('data-autofill', newV);
        }
        if(newV!=='0.00') inf.placeholder=newV+' Hz'; // wskazówka zamiast zerowania
      }
    }
  }catch(e){ console.error('fetchRegs err',e); }
}
function set(id,v){ const e=document.getElementById(id); if(e) e.textContent=v; }
async function cmd(c,v){
  if(logicalActiveList.indexOf(currentSid)<0){ alert('Wybrany SID nieaktywny'); return; }
  let u='/inverter_master/cmd?sid='+currentSid+'&c='+encodeURIComponent(c);
  if(v!==undefined) u+='&v='+encodeURIComponent(v);
  try{
    const r=await fetch(u).then(r=>r.json().catch(()=>({ok:true})));
    if(r && r.error) alert('CMD error: '+r.error);
  }catch(e){ console.error('cmd error',e); }
}
function setFreq(){
  const inf=document.getElementById('in_freq');
  const v=inf.value;
  if(!v) return;
  cmd('setf',v);
}
function attachFreqGuards(){
  const inf=document.getElementById('in_freq');
  if(!inf) return;
  inf.addEventListener('focus', ()=>{ freqEditing=true; freqEditTs=Date.now(); });
  inf.addEventListener('input', ()=>{ freqEditing=true; freqEditTs=Date.now(); });
  inf.addEventListener('blur',  ()=>{ freqEditing=false; freqEditTs=Date.now(); });
}
async function loadCfg(){
  try{
    const c=await fetch('/inverter_master/config').then(r=>r.json());
    if(!c) return;
    document.getElementById('rtu_baud').value=c.rtu.baud;
    document.getElementById('rtu_par').value=c.rtu.par;
    document.getElementById('rtu_poll').value=c.rtu.poll;
    document.getElementById('mqtt_topic').value=c.mqtt.topic;
    document.getElementById('mqtt_period').value=c.mqtt.period;
    document.getElementById('mqtt_freq_rate').value=c.mqtt.freq_rate;
  }catch(e){ console.error('loadCfg err',e); }
}
async function saveCfg(){
  const p=new URLSearchParams();
  p.append('baud',document.getElementById('rtu_baud').value);
  p.append('par',document.getElementById('rtu_par').value);
  p.append('poll',document.getElementById('rtu_poll').value);
  p.append('topic',document.getElementById('mqtt_topic').value);
  p.append('period',document.getElementById('mqtt_period').value);
  p.append('freq_rate',document.getElementById('mqtt_freq_rate').value);
  try{
    const r=await fetch('/inverter_master/config',{method:'POST',body:p}).then(r=>r.json());
    if(r) document.getElementById('cfg_status').textContent='Zapisano (rtu_changed='+r.rtu_changed+', mqtt_changed='+r.mqtt_changed+')';
  }catch(e){ console.error('saveCfg err',e); }
}
async function loadRS485(){
  try{
    const d=await fetch('/inverter_master/rs485').then(r=>r.json());
    if(!d) return;
    const box=document.getElementById('rs485_table');
    let h='<table><tr><th>SID</th><th>Active</th><th>Miss</th><th>R_OK</th><th>R_FAIL</th><th>W_OK</th><th>W_FAIL</th><th>Backoff(ms)</th><th>NextProbe(ms)</th></tr>';
    d.sid_stats.forEach(s=>{
      h+=`<tr><td>${s.sid}</td><td>${s.active}</td><td>${s.miss}</td><td>${s.read_ok}</td><td>${s.read_fail}</td><td>${s.write_ok}</td><td>${s.write_fail}</td><td>${s.backoff_ms}</td><td>${s.next_probe_ms}</td></tr>`;
    });
    h+='</table>';
    box.innerHTML=h;
    const summary=document.getElementById('rs485_summary');
    summary.textContent=`Baud=${d.rtu.baud} Par=${d.rtu.par} Poll=${d.rtu.poll}ms Queue=${d.queue_len} RC(success=${d.rc_counts["00"]},timeout=${d.rc_counts["E2"]},crc=${d.rc_counts["E3"]},queue_full=${d.rc_counts["FE"]},rate_limited=${d.rc_counts["F0"]})`;
  }catch(e){ console.error('loadRS485 err',e); }
}
function startLoops(){
  if(pollTimer) clearInterval(pollTimer);
  pollTimer=setInterval(fetchRegs,1000);
  fetchRegs();
  if(rs485Timer) clearInterval(rs485Timer);
  rs485Timer=setInterval(()=>{
    loadActive();
    loadRS485();
  },4000);
  loadActive();
  loadRS485();
  attachFreqGuards();
}
window.onload=()=>{
  loadCfg();
  startLoops();
};
</script>
</head><body>
<div class="anchor-bar">
  <a href="#status" onclick="anchorScroll('status')">Status & SID</a>
  <a href="#control" onclick="anchorScroll('control')">Sterowanie</a>
  <a href="#regs" onclick="anchorScroll('regs')">Rejestry RAW</a>
  <a href="#config" onclick="anchorScroll('config')">Konfiguracja</a>
  <a href="#rs485" onclick="anchorScroll('rs485')">Diagnostyka RS485</a>
  <a href="/mqtt/repub">MQTT topics</a>
  <!-- DODANE: przycisk do strony głównej -->
  <a href="/">Strona główna</a>
</div>
<div class="container">

  <h2 id="status">Status & SID <span class="badge">v21a</span></h2>
  <div class="card">
    <div class="small-note">Tabela SID (radio). Pola sterowania aktywne tylko gdy RS485 raportuje Active=true.</div>
    <div id="sid_table_host" style="margin-top:8px"></div>
  </div>

  <h2 id="control">Sterowanie (SID: <span id="current_sid_ctrl">1</span>)</h2>
  <div class="card control-section">
    <div class="flex-row">
      <button class="btn btn-ok" onclick="cmd('start')">START</button>
      <button class="btn btn-danger" onclick="cmd('stop')">STOP</button>
      <button class="btn" onclick="cmd('jog')">JOG</button>
      <button class="btn btn-alt" onclick="cmd('reset')">RESET</button>
    </div>
    <div class="flex-row">
      <button class="btn" onclick="cmd('dir','fwd')">FWD</button>
      <button class="btn" onclick="cmd('dir','rev')">REV</button>
      <button class="btn" onclick="cmd('dir','chg')">CHANGE</button>
    </div>
    <div class="flex-row">
      <input id="in_freq" type="number" step="0.01" min="0" max="400" placeholder="Hz">
      <button class="btn btn-ok" onclick="setFreq()">Set Freq</button>
    </div>
    <div class="small-note">Pole nie będzie nadpisywane podczas edycji; aktualizuje się z set_reported gdy nie edytujesz.</div>
  </div>

  <h2 id="regs">Rejestry RAW (SID: <span id="current_sid_regs">1</span>)</h2>
  <div class="card">
    <table>
      <tr><th>Register</th><th>Raw</th><th>Scaled</th></tr>
      <tr><td>warn_fault</td><td id="rw_warn">0</td><td>-</td></tr>
      <tr><td>drive_status</td><td id="rw_dstat">0</td><td>-</td></tr>
      <tr><td>set_reported</td><td id="rw_setr">0</td><td id="sc_setr">0.00 Hz</td></tr>
      <tr><td>out_freq</td><td id="rw_fout">0</td><td id="sc_fout">0.00 Hz</td></tr>
      <tr><td>out_curr</td><td id="rw_iout">0</td><td id="sc_iout">0.00 A</td></tr>
      <tr><td>dc_bus</td><td id="rw_vdc">0</td><td id="sc_vdc">0.0 V</td></tr>
      <tr><td>out_volt</td><td id="rw_vout">0</td><td id="sc_vout">0.0 V</td></tr>
      <tr><td>ctrl_word</td><td id="rw_ctrl">0</td><td>-</td></tr>
      <tr><td>set_freq</td><td id="rw_setf">0</td><td>-</td></tr>
      <tr><td>flags</td><td id="rw_flags">0</td><td>-</td></tr>
    </table>
  </div>

  <h2 id="config">Konfiguracja RTU / MQTT</h2>
  <div class="card">
    <div class="flex-row">
      <label class="hl">RTU Baud</label>
      <select id="rtu_baud">
        <option value="9600">9600</option><option value="19200">19200</option>
        <option value="38400">38400</option><option value="57600">57600</option>
        <option value="115200">115200</option>
      </select>
      <label class="hl">Parity</label>
      <select id="rtu_par">
        <option value="0">NONE</option><option value="1">EVEN</option><option value="2">ODD</option>
      </select>
      <label class="hl">Poll(ms)</label>
      <input id="rtu_poll" type="number" min="100" max="5000" style="width:90px">
    </div>
    <hr>
    <div class="flex-row">
      <label class="hl">MQTT Topic</label><input id="mqtt_topic" type="text" style="width:240px">
      <label class="hl">Period(ms)</label><input id="mqtt_period" type="number" min="1000" max="3600000" style="width:120px">
      <label class="hl">FreqRate(/min)</label><input id="mqtt_freq_rate" type="number" min="0" max="60" style="width:80px">
    </div>
    <div class="flex-row">
      <button class="btn" onclick="saveCfg()">Zapisz</button>
      <span id="cfg_status" class="small-note"></span>
    </div>
  </div>

  <h2 id="rs485">Diagnostyka RS485</h2>
  <div class="card">
    <div id="rs485_summary" class="small-note">Ładowanie...</div>
    <div id="rs485_table" style="margin-top:8px"></div>
  </div>

</div>
</body></html>
)HTML";

static AutoMultiInverter _auto;

extern "C" void inverter_master_begin(){ _auto.begin(); }
extern "C" bool inverter_rtu_apply(uint8_t /*sid_unused*/,uint32_t baud,uint8_t parity,uint16_t pollMs){
  return _auto.applyRTU(baud,parity,pollMs);
}
extern "C" uint32_t inverter_get_last_state_pub(){ return _auto.getLastStatePub(); }
extern "C" uint32_t inverter_get_last_decode_pub(){ return _auto.getLastDecodePub(); }