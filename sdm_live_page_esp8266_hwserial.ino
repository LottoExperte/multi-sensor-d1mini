//sdm live page example by reaper7

#define READSDMEVERY  3000                                                      //read sdm every 3000ms
#define NBREG   9   
#define NBREG1  5  
#define NBREG2  25                                                               //number of sdm registers to read
//#define USE_STATIC_IP

/*  WEMOS D1 Mini
                     ______________________________
                    |   L T L T L T L T L T L T    |
                    |                              |
                 RST|                             1|TX HSer
                  A0|                             3|RX HSer
                  D0|16                           5|D1
                  D5|14                           4|D2
                  D6|12                    10kPUP_0|D3
RX SSer/HSer swap D7|13                LED_10kPUP_2|D4
TX SSer/HSer swap D8|15                            |GND
                 3V3|__                            |5V
                       |                           |
                       |___________________________|
*/
#include <ESP_EEPROM.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <ESPAsyncTCP.h>                                                        //https://github.com/me-no-dev/ESPAsyncTCP
#include <ESPAsyncWebServer.h>                                                  //https://github.com/me-no-dev/ESPAsyncWebServer

#include <SoftwareSerial.h>
#include <SDM.h>                                                                //https://github.com/reaper7/SDM_Energy_Meter

#include <string>
#include <sstream>
#include "topnavicss.h"

//#if !defined ( USE_HARDWARESERIAL )
//  #error "This example works with Hardware Serial on esp8266, please uncomment #define USE_HARDWARESERIAL in SDM_Config_User.h"
//#endif
//------------------------------------------------------------------------------

AsyncWebServer server(80);

SoftwareSerial swSerSDM(D2, D3);

SDM sdm(swSerSDM, 9600, D1); 

//SDM sdm(swSerSDM, SWSERIAL_8N1, DERE_PIN, SDM_UART_CONFIG, false);                            //HARDWARE SERIAL

//------------------------------------------------------------------------------
String devicename = "PWRMETER";

IPAddress ip(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

const char* wifiName1 = "SENSOR";
const char* wifiPass1 = "hoffmann";

const char* wifi_ssid = "SENSOR";
const char* wifi_password = "hoffmann";

String lastresetreason = "";
static String wifi_Name = wifi_ssid;
static String wifi_Pass = wifi_password;
uint64_t zaehlerstand = 0;
uint64_t zaehlerstand2 = 0;
uint64_t zaehlerstand3 = 0;
float einzel_abruf = NAN;
uint8_t einzel_addr = 0;
int einzel_register = 0;
float einzel2_abruf = NAN;
uint8_t einzel2_addr = 0;
int einzel2_register = 0;
int READSDMEVERY_C = READSDMEVERY;
uint8_t charsRead = 0;
char rxBuffer [20];
String smlMessage [50];
String rxID;
String ServerID = "UNKNOWN";
String SerienNr = "UNKNOWN";
String consSum = "0";
String BezugT1 = "0";
String BezugT2 = "0";
String powerL1 = "0";
String powerL2 = "0";
String powerL3 = "0";
String powerSum = "0";
String lieferSum = "0";
String mtrState = "UNK";
String sMessage = "";
byte incomingByte;
int leseindex = 0;
int lesezeile = 0;
uint32_t m_timeout = 1000;

unsigned long readtime;
//------------------------------------------------------------------------------

typedef struct {
  float regvalarr[5];
  const uint16_t regarr;
  const char regnamen[30];
  const char regeinheit[6];  
} sdm_struct;
//------------------SDM630------------------------------------------------------------
sdm_struct sdmarr2[NBREG2] = {
  {{0.00,0.00,0.00,0.00,0.00}, SDM_SUM_LINE_CURRENT,"CurrentSUM","A"},                                    //A
  {{0.00,0.00,0.00,0.00,0.00}, SDM_PHASE_1_POWER,"PowerL1","W"},                                          //W
  {{0.00,0.00,0.00,0.00,0.00}, SDM_PHASE_2_POWER,"PowerL2","W"},                                          //W
  {{0.00,0.00,0.00,0.00,0.00}, SDM_PHASE_3_POWER,"PowerL3","W"},                                          //W
  {{0.00,0.00,0.00,0.00,0.00}, SDM_PHASE_1_APPARENT_POWER,"BlindleistungL1","W"},                         //W
  {{0.00,0.00,0.00,0.00,0.00}, SDM_PHASE_2_APPARENT_POWER,"BlindleistungL2","W"},                         //W
  {{0.00,0.00,0.00,0.00,0.00}, SDM_PHASE_3_APPARENT_POWER,"BlindleistungL3","W"},                         //W  
  {{0.00,0.00,0.00,0.00,0.00}, SDM_TOTAL_SYSTEM_POWER,"TotalSystemPower","W"},                                    //W
  {{0.00,0.00,0.00,0.00,0.00}, SDM_MAXIMUM_TOTAL_SYSTEM_POWER_DEMAND,"PowerMAX","W"},                     //W 
  {{0.00,0.00,0.00,0.00,0.00}, SDM_TOTAL_SYSTEM_POWER_FACTOR,"PFTOTAL",""},                               //PF
  {{0.00,0.00,0.00,0.00,0.00}, SDM_FREQUENCY,"FREQUENCY","Hz"},                                            //Hz
  {{0.00,0.00,0.00,0.00,0.00}, SDM_IMPORT_ACTIVE_ENERGY,"ImportEnergie","kWh"},                              //kWh
  {{0.00,0.00,0.00,0.00,0.00}, SDM_EXPORT_ACTIVE_ENERGY,"ExportEnergie","kWh"},                              //kWh  
  {{0.00,0.00,0.00,0.00,0.00}, SDM_TOTAL_ACTIVE_ENERGY,"TotalEnergie","kWh"},                                //kWh
  {{0.00,0.00,0.00,0.00,0.00}, SDM_LINE_1_TO_LINE_2_VOLTS,"VoltageL1L2","V"},                             //V
  {{0.00,0.00,0.00,0.00,0.00}, SDM_LINE_2_TO_LINE_3_VOLTS,"VoltageL2L3","V"},                             //V
  {{0.00,0.00,0.00,0.00,0.00}, SDM_LINE_3_TO_LINE_1_VOLTS,"VoltageL3L1","V"},                             //V
  {{0.00,0.00,0.00,0.00,0.00}, SDM_TOTAL_SYSTEM_REACTIVE_POWER,"BlindleistSUM","kWh"},                   //VAr
  {{0.00,0.00,0.00,0.00,0.00}, SDM_TOTAL_SYSTEM_APPARENT_POWER,"ScheinleistSUM","kWh"},                   //VA
  {{0.00,0.00,0.00,0.00,0.00}, SDM_L1_IMPORT_ACTIVE_ENERGY,"ImportL1","kWh"},                               //kWh
  {{0.00,0.00,0.00,0.00,0.00}, SDM_L2_IMPORT_ACTIVE_ENERGY,"ImportL2","kWh"},                               //kWh
  {{0.00,0.00,0.00,0.00,0.00}, SDM_L3_IMPORT_ACTIVE_ENERGY,"ImportL3","kWh"},                                //kWh
  {{0.00,0.00,0.00,0.00,0.00}, SDM_L1_EXPORT_ACTIVE_ENERGY,"ExportL1","kWh"},                               //kWh
  {{0.00,0.00,0.00,0.00,0.00}, SDM_L2_EXPORT_ACTIVE_ENERGY,"ExportL2","kWh"},                               //kWh
  {{0.00,0.00,0.00,0.00,0.00}, SDM_L3_EXPORT_ACTIVE_ENERGY,"ExportL3","kWh"}                                //kWh
};
//------------------SDM72D-M-----------------------------------------------------------

sdm_struct sdmarr[NBREG] = {
  {{0.00,0.00,0.00,0.00,0.00}, SDM_TOTAL_SYSTEM_POWER,"TOTAL_SYSTEM_POWER", "W"},                                                  //V
  {{0.00,0.00,0.00,0.00,0.00}, SDM_IMPORT_ACTIVE_ENERGY,"IMPORT_ACTIVE_ENERGY", "kWh"},                                                  //A
  {{0.00,0.00,0.00,0.00,0.00}, SDM_EXPORT_ACTIVE_ENERGY,"EXPORT_ACTIVE_ENERGY", "kWh"},                                                    //W
  {{0.00,0.00,0.00,0.00,0.00}, SDM_TOTAL_ACTIVE_ENERGY, "TOTAL_ACTIVE_ENERGY", "kWh"},                                           //PF
  {{0.00,0.00,0.00,0.00,0.00}, SDM_CURRENT_RESETTABLE_TOTAL_ACTIVE_ENERGY, "RESET_TOTAL_ACTIVE_ENERGY",  "kWh"},                                                        //Hz
  {{0.00,0.00,0.00,0.00,0.00}, SDM_CURRENT_RESETTABLE_IMPORT_ENERGY,"RESET_IMPORT_ENERGY",  "kWh"},
  {{0.00,0.00,0.00,0.00,0.00}, SDM_CURRENT_RESETTABLE_EXPORT_ENERGY, "RESET_EXPORT_ENERGY",  "kWh"},
  {{0.00,0.00,0.00,0.00,0.00}, SDM_IMPORT_POWER, "IMPORT_POWER", "W"},
  {{0.00,0.00,0.00,0.00,0.00}, SDM_EXPORT_POWER, "EXPORT_POWER", "W"}
};
//------------------SDM72D-M-----------------------------------------------------------

//------------------SDMXXX-----------------------------------------------------------
sdm_struct sdmarr1[NBREG1] = {
  {{0.00,0.00,0.00,0.00,0.00}, SDM_IMPORT_ACTIVE_ENERGY, "IMPORT_ACTIVE_ENERGY", "kWh"},                                                  //V
  {{0.00,0.00,0.00,0.00,0.00}, SDM_EXPORT_ACTIVE_ENERGY, "EXPORT_ACTIVE_ENERGY", "kWh"},                                                  //A
  {{0.00,0.00,0.00,0.00,0.00}, SDM_TOTAL_ACTIVE_ENERGY, "TOTAL_ACTIVE_ENERGY", "kWh"},                                                    //W
  {{0.00,0.00,0.00,0.00,0.00}, SDM_TOTAL_REACTIVE_ENERGY, "TOTAL_REACTIVE_ENERGY", "kVArh"},                                             //PF
  {{0.00,0.00,0.00,0.00,0.00}, SDM_FREQUENCY,"FREQUENCY", "Hz"}                                                         //Hz
};
//------------------SDMXXX---------------------------------------------------------




struct modebusstruk {
  short   addr;  
  char    device[8] = "";
};

struct MyEEPROMStruct {
  int     langssid;
  char    strSSID[50] = "";
  int     langpass;  
  char    strPASS[50] = "";
  char    hostname[20] = "";
  modebusstruk  modbus[5];   
  uint64_t    counter;    
  uint64_t    counter2;  
  uint64_t    counter3;  
  boolean state;
  } eepromVar;

void cleareprom() {
  EEPROM.begin(2*sizeof(MyEEPROMStruct));
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit(); 
}

bool readFromMemory(){
  int addr = 0;
  EEPROM.begin(2*sizeof(MyEEPROMStruct));
  int templang;
  if (EEPROM.percentUsed()>=0) {
    EEPROM.get(addr,eepromVar);    
    wifi_Name = String(eepromVar.strSSID);
    wifi_Name.trim(); 
    Serial.println("SSIP:"+wifi_Name);  //Print the text  
    wifi_Pass = String(eepromVar.strPASS);  
    wifi_Pass.trim();
    Serial.println("Passwort:" + wifi_Pass);  //Print the text 
    devicename = String(eepromVar.hostname);  
    devicename.trim();
    Serial.println("Device:" + devicename);  //Print the text  
    zaehlerstand = eepromVar.counter;
    zaehlerstand2 = eepromVar.counter2;
    zaehlerstand3 = eepromVar.counter3;    
    Serial.print("Zaehler:");  //Print the text      
    Serial.println(zaehlerstand);  //Print the text     

    return true;
  } else {return false;}
  EEPROM.end();  
  READSDMEVERY_C=0;
  for (int i = 0; i<5; i++) {if (eepromVar.modbus[i].addr>0) READSDMEVERY_C++;}
  READSDMEVERY_C=READSDMEVERY_C*READSDMEVERY;
}

void writeToMemory(String ssid, String pass, String hostn, uint64_t count, uint64_t count2, uint64_t count3){
  int addr = 0;
  Serial.println("");
  int templang;
  templang=ssid.length();
  if (templang!=0) {    
    ssid.toCharArray(eepromVar.strSSID,50);
    pass.toCharArray(eepromVar.strPASS,50); 
    hostn.toCharArray(eepromVar.hostname,20); 
    eepromVar.langssid = templang;   
    eepromVar.langpass = pass.length();
    eepromVar.counter = count;
    eepromVar.counter2 = count2;   
    eepromVar.counter3 = count3;     
    EEPROM.begin(2*sizeof(MyEEPROMStruct));
    EEPROM.put(addr,eepromVar);
    EEPROM.commit();
	  delay(3000);    
	  ESP.restart();     
  }
}

void xmlrequest(AsyncWebServerRequest *request) {
  String XML = F("<?xml version='1.0'?><xml>");
  int paramsNr = request->params();
  int adresse = 0;
  String device = "SDM72D";
  for(int i=0;i<paramsNr;i++){
    AsyncWebParameter* p = request->getParam(i);
    Serial.print("Param name: ");
    Serial.println(p->name());
    Serial.print("Param value: ");
    Serial.println(p->value());
    Serial.println("------");
    String merkeA = p->name();
    merkeA.trim();    
    if (merkeA=="DEV") {
      device = p->value();
    } else {
     if (merkeA=="ADDR") {
      adresse = p->value().toInt();
     }  
    }     
  }
  String merke = eepromVar.modbus[adresse].device; merke.trim();    
  
  if (merke == "SDM72D") {
       XML += "<response0>";
      XML += "device";
      XML += "</response0>";
      XML += "<bezeichnung0>";
      XML += "SDM72D";
      XML += "</bezeichnung0>";
      for (int i = 1; i < NBREG+1; i++) {
        XML += "<response" + String(i) + ">";
        XML += String(sdmarr[i-1].regvalarr[adresse],2);
        XML += "</response" + String(i) + ">";
        XML += "<bezeichnung" + String(i) + ">";
        XML += String(sdmarr[i-1].regnamen);
        XML += "</bezeichnung" + String(i) + ">";
        XML += "<einheit" + String(i) + ">";
        XML += String(sdmarr[i-1].regeinheit);
        XML += "</einheit" + String(i) + ">";        
      }
  }

  if (merke == "SDMXXX") {
       XML += "<response0>";
      XML += "device";
      XML += "</response0>";
      XML += "<bezeichnung0>";
      XML += "SDMXXX";
      XML += "</bezeichnung0>";   
      for (int i = 1; i < NBREG1+1; i++) {
        XML += "<response" + String(i) + ">";
        XML += String(sdmarr1[i-1].regvalarr[adresse],2);
        XML += "</response" + String(i) + ">";
        XML += "<bezeichnung" + String(i) + ">";
        XML += String(sdmarr1[i-1].regnamen);
        XML += "</bezeichnung" + String(i) + ">";
        XML += "<einheit" + String(i) + ">";
        XML += String(sdmarr1[i-1].regeinheit);
        XML += "</einheit" + String(i) + ">";
      }
  }

  if (merke == "SDM630") {
       XML += "<response0>";
      XML += "device";
      XML += "</response0>";
      XML += "<bezeichnung0>";
      XML += "SDM630";
      XML += "</bezeichnung0>";
    for (int i = 1; i < NBREG2+1; i++) {
      XML += "<response" + String(i) + ">";
      XML += String(sdmarr2[i-1].regvalarr[adresse],2);
      XML += "</response" + String(i) + ">";
      XML += "<bezeichnung" + String(i) + ">";
      XML += String(sdmarr2[i-1].regnamen);
      XML += "</bezeichnung" + String(i) + ">";
      XML += "<einheit" + String(i) + ">";
      XML += String(sdmarr2[i-1].regeinheit);
      XML += "</einheit" + String(i) + ">";      
    }
  }
//  }
  XML += F("<freeh>");
  XML += String(ESP.getFreeHeap());
  XML += F("</freeh>");
  XML += F("<rst>");
  XML += lastresetreason;
  XML += F("</rst>");
  XML += F("<count>");
  if (digitalRead(D4) == LOW) {XML += "geschlossen";} else {XML += "offen";}
  XML += F("</count>");
  XML += F("<stand>");
  XML += String(zaehlerstand);
  XML += F("</stand>");
  XML += F("<count2>");
  if (digitalRead(D7) == LOW) {XML += "geschlossen";} else {XML += "offen";}
  XML += F("</count2>");
  XML += F("<stand2>");
  XML += String(zaehlerstand2);
  XML += F("</stand2>");  
  XML += F("<count3>");
  if (digitalRead(D8) == LOW) {XML += "geschlossen";} else {XML += "offen";}
  XML += F("</count3>");  
  XML += F("<stand3>");
  XML += String(zaehlerstand3);
  XML += F("</stand3>"); 
  XML += F("<einzelwert1>");
  XML += String(einzel_abruf);
  XML += F("</einzelwert1>");  
  XML += F("<einzelwert2>");
  XML += String(einzel2_abruf);
  XML += F("</einzelwert2>");  
    
  XML += F("</xml>");
  request->send(200, "text/xml", XML);
}
//------------------------------------------------------------------------------

void jsonrequest(AsyncWebServerRequest *request) {
  String webPage; 
  StaticJsonDocument<2048> doc;
  int paramsNr = request->params();
  int adresse = 0;
  String device = "SDM72D";
  for(int i=0;i<paramsNr;i++){
    AsyncWebParameter* p = request->getParam(i);
    Serial.print("Param name: ");
    Serial.println(p->name());
    Serial.print("Param value: ");
    Serial.println(p->value());
    Serial.println("------");
    String merkeA = p->name();
    merkeA.trim();    
    if (merkeA=="DEV") {
      device = p->value();
    } else {
     if (merkeA=="ADDR") {
      adresse = p->value().toInt();
     }  
    }     
  }
  String merke = eepromVar.modbus[adresse].device; merke.trim();  
  if (merke == "SDM630") {
    doc["device"] = "SDM630_"+ String(adresse);
    doc["sensor"] = devicename;
    doc["divisor"] = "100";
    for (int i = 0; i < NBREG2; i++) {
        doc[String(sdmarr2[i].regnamen)] = String(sdmarr2[i].regvalarr[adresse]*100,0);
      }
  }
  
  if (merke == "SDM72D") {
      doc["device"] = "SDM72D_"+String(adresse);
      doc["sensor"] = devicename;
      doc["divisor"] = "100";
      for (int i = 0; i < NBREG; i++) {
        doc[String(sdmarr[i].regnamen)] = String(sdmarr[i].regvalarr[adresse]*100,0);
      }
  }

  if (merke == "SDMXXX") {
      doc["device"] = "SDMXXX_"+String(adresse);
      doc["sensor"] = devicename;
      doc["divisor"] = "100";
      for (int i = 0; i < NBREG; i++) {
        doc[String(sdmarr1[i].regnamen)] = String(sdmarr1[i].regvalarr[adresse]*100,0);
      }
  }
 // }
  doc["Zaehlerstand1"] = String(zaehlerstand);
  doc["Zaehlerstand2"] = String(zaehlerstand2);  
  doc["Zaehlerstand3"] = String(zaehlerstand3);  
  if (digitalRead(D4) == LOW) {doc["Zaehler1status"] = "geschlossen";} else {doc["Zaehler1status"] = "offen";}
  if (digitalRead(D7) == LOW) {doc["Zaehler2status"] = "geschlossen";} else {doc["Zaehler2status"] = "offen";}
  if (digitalRead(D8) == LOW) {doc["Zaehler3tatus"] = "geschlossen";} else {doc["Zaehler3status"] = "offen";}  
  if (einzel_addr!=0) doc["Einzelwert1"] = String(einzel_abruf);  
  if (einzel2_addr!=0) doc["Einzelwert2"] = String(einzel2_abruf);    
  serializeJson(doc, webPage);
  request->send(200, "text/plane", webPage);
}

//------------------------------------------------------------------------------
void jsonrequestz(AsyncWebServerRequest *request) {
  String webPage; 
  StaticJsonDocument<255> doc;

  doc["zaehler1"] = String(zaehlerstand);
  if (digitalRead(D4) == LOW) {doc["Zaehler1status"] = "geschlossen";} else {doc["Zaehler1status"] = "offen";}  
  doc["zaehler2"] = String(zaehlerstand2);  
  if (digitalRead(D7) == LOW) {doc["Zaehler2status"] = "geschlossen";} else {doc["Zaehler2status"] = "offen";}    
  doc["zaehler2"] = String(zaehlerstand3);  
  if (digitalRead(D8) == LOW) {doc["Zaehler2status"] = "geschlossen";} else {doc["Zaehler2status"] = "offen";}    

  serializeJson(doc, webPage);
  request->send(200, "text/plane", webPage);
}
//------------------------------------------------------------------------------

void registerdirect(AsyncWebServerRequest *request) {
  
  String webPage = "";
  int wertnr = 1;
  int paramsNr = request->params();
  String merkeA = "";
  for(int i=0;i<paramsNr;i++){
    AsyncWebParameter* p = request->getParam(i);
    Serial.print("Param name: ");
    Serial.println(p->name());
    Serial.print("Param value: ");
    Serial.println(p->value());
    Serial.println("------");
    merkeA = p->name();
    merkeA.trim();    
    if (merkeA=="ADDR") {
      einzel_addr = p->value().toInt();
    } else {
      if (merkeA == "WERT") {
        wertnr = p->value().toInt();
      }
      if (merkeA == "REG") {
        einzel_register = p->value().toInt();
        webPage = "ADDR: " + String(einzel_addr) + "  Register: 0x" + String(einzel_register,HEX)+" ("+String(einzel_register) + ")   ";       
      } else {if (merkeA=="ADDR2") {
                einzel2_addr = p->value().toInt();
              } else {
                  if (merkeA == "REG2") {
                      einzel2_register = p->value().toInt();
                  } 
              }
            }
    }   
  }
  if (wertnr==1) {
  if ((einzel_addr==0) or (einzel_addr>255)) {
    einzel_addr = 0; einzel_register=NAN;
     String webPage = "Übergeben Sie Parameter mit der ModBus Adresse (ADDR) und der Registernummer (REG)! ";
  } else  {
  webPage += String(einzel_abruf);
  }
  }
  if ((einzel2_addr!=0) && (wertnr==2)) {
  webPage += String(einzel2_abruf);
  }  
  request->send(200, "text/plane", webPage);    
}
  

void tabelleinbinden(AsyncWebServerRequest *request) {
  int paramsNr = request->params();
  int devisenr = 0;
  String device = "SDM72D";
  for(int i=0;i<paramsNr;i++){
    AsyncWebParameter* p = request->getParam(i);
    Serial.print("Param name: ");
    Serial.println(p->name());
    Serial.print("Param value: ");
    Serial.println(p->value());
    Serial.println("------");
    String merkeA = p->name();
    merkeA.trim();    
    if (merkeA=="DEV") {
      devisenr = p->value().toInt();
    }    
  }
  device = eepromVar.modbus[devisenr].device; device.trim();
  String merke = eepromVar.modbus[devisenr].device; merke.trim();    
  int feldanzahl = 9;
  if (merke == "SDM72D") { feldanzahl = NBREG;}
    else { if (merke == "SDM630") { feldanzahl = NBREG2;} 
         else {feldanzahl = NBREG1;}
    }
  String webPage1 = "<TABLE width=100% BORDER=1>";
  for (int i=0; i<feldanzahl; i++) {    
      if (feldanzahl==9) {     
		  webPage1 +="<TR><TH title='Zeile" + String(i) + "'>"+String(sdmarr[i].regnamen)+"</TH><TD>"+String(sdmarr[i].regvalarr[devisenr],2) + "</TD><TD>"+String(sdmarr[i].regeinheit) + "</TD></TR>";
      } else { if (feldanzahl==23) {
	  webPage1 +="<TR><TH title='Zeile" + String(i) + "'>"+String(sdmarr2[i].regnamen)+"</TH><TD>"+String(sdmarr2[i].regvalarr[devisenr],2) + "</TD><TD>"+String(sdmarr2[i].regeinheit) + "</TD></TR>";
      } else {
	  webPage1 +="<TR><TH title='Zeile" + String(i) + "'>"+String(sdmarr1[i].regnamen)+"</TH><TD>"+String(sdmarr1[i].regvalarr[devisenr],2) + "</TD><TD>"+String(sdmarr1[i].regeinheit) + "</TD></TR>";

      }
      }
    }    
	webPage1 += "<TR><TH title='Zustand'>ZAEHLER_Stand</TH><TD colspan='2'>"+String(zaehlerstand)+"</TD></TR>";
  webPage1 += "<TR><TH title='Zustand2'>ZAEHLER2_Stand</TH><TD colspan='2'>"+String(zaehlerstand2)+"</TD></TR>";
  webPage1 += "<TR><TH title='Zustand3'>ZAEHLER3_Stand</TH><TD colspan='2'>"+String(zaehlerstand3)+"</TD></TR>";;
	webPage1 += "<TR><TH title='FREE HEAP'>FREE HEAP</TH><TD>"+String(ESP.getFreeHeap())+"</TD><TD>bytes</TD></TR>";
	webPage1 += "<TR><TH title='LAST RESET REASON'>LAST RESET REASON</TH><TD colspan='2'>"+lastresetreason+"</A></TD></TR>";
  webPage1 += "</TABLE>";
  request->send(200, "text/html", webPage1);
}


void indexrequest(AsyncWebServerRequest *request) {

  int paramsNr = request->params();
  int devisenr = 0;
  String device = "SDM72D";
  for(int i=0;i<paramsNr;i++){
    AsyncWebParameter* p = request->getParam(i);
    Serial.print("Param name: ");
    Serial.println(p->name());
    Serial.print("Param value: ");
    Serial.println(p->value());
    Serial.println("------");
    String merkeA = p->name();
    merkeA.trim();    
    if (merkeA=="DEV") {
      devisenr = p->value().toInt();
    }    
  }
  device = eepromVar.modbus[devisenr].device; device.trim();
  String merke = eepromVar.modbus[devisenr].device; merke.trim();    
  int feldanzahl = NBREG;
  if (merke == "SDM72D") { feldanzahl = NBREG;}
    else { if (merke == "SDM630") { feldanzahl = NBREG2;} 
         else {feldanzahl = NBREG1;}
    } 
  String webPage = "<!DOCTYPE HTML><html  lang='de'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  webPage += "<script>";
  webPage += "window.setInterval('reloadIFrame();', 5000);";
  webPage += "function reloadIFrame() {";
  webPage += "document.getElementById('tabelle').src=tabelle.src;";
  webPage += "};";
  webPage += "function easyload() {";
  webPage += "document.getElementById('tabelle').src='/easy';";
  webPage += "}";
  webPage += "</script>";
  webPage += "<link rel='stylesheet' href='https://www.hoffmann-software.de/iot.css'>";

  String substr = "<DEVISENR>";
  String merkeA = String(devisenr);
  webPage.replace(substr,merkeA);
  substr = "<FELDZAHL>";
  merkeA = String(feldanzahl);
  webPage.replace(substr, merkeA);
  webPage += "</head><body  onload='process()'><CENTER>";
  webPage += "<div class='header'>";
  webPage += "<h2>Hoffmanns Multi-Sensor - "+ device + " Werteübersicht</h2>";
  webPage += devicename +" aktuell mit " + WiFi.SSID() + " (" + String(WiFi.RSSI()) +", C" + String(WiFi.channel())  +") und der IP: " + WiFi.localIP().toString() +" verbunden. <br><br>";
  webPage += "</div>"; 
  webPage += TOPNAVI;
  webPage += "<div class='topnav'> <a href='/'>"+String(eepromVar.modbus[0].device)+"</a>  <a href='/?DEV=1'>"+eepromVar.modbus[1].device+"</a> <a href='/?DEV=2'>"+eepromVar.modbus[2].device+"</a> <a href='/?DEV=3'>"+eepromVar.modbus[3].device+"</a> <a href='/?DEV=4'>"+eepromVar.modbus[4].device+"</a> <a href=# onclick=easyload()>EasyMeter S0</a></div>";
  webPage += " <CENTER>";
  webPage += "<iframe id='tabelle' src=/tabelle?DEV=" + String(devisenr) + " style='border-width:0' width=100% height=1000 frameborder='0' scrolling='no'></iframe>";  
  webPage += "<br><br>";
  webPage += "</CENTER></body></html>";
  request->send(200, "text/html", webPage);
}
//------------------------------------------------------------------------------


void Zaehler_setzen(AsyncWebServerRequest *request) {

  String webPage = "<!DOCTYPE HTML><html  lang=\"de\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  webPage += "<link rel='stylesheet' href='https://www.hoffmann-software.de/iot.css'>";
  webPage += "</head><body><CENTER>";
  webPage += "<div class=\"header\">";
  webPage += "<h2>Hoffmanns Multi-Sensor - Zählerstände</h2>";
  webPage += devicename +" aktuell mit " + WiFi.SSID() + " (" + String(WiFi.RSSI()) +", C" + String(WiFi.channel())  +") und der IP: " + WiFi.localIP().toString() +" verbunden. <br><br>";
  webPage += "</div>"; 
  webPage += TOPNAVI;
  webPage += "<br>Tragen Sie den Zählerstand ein ab dem der Sensor weiterzählen soll.<br><br>";
  webPage += "<form action=/ method=POST>";
  webPage += "<label>Zählerstand </label><input type=numeric name=counter maxlength=20 value=";
  webPage += String(zaehlerstand);
  webPage += "><br><br>";
  webPage += "<label>Zählerstand2 </label><input type=numeric name=counter2 maxlength=20 value=";
  webPage += String(zaehlerstand2);
  webPage += "><br><br>";  
  webPage += "<label>Zählerstand3 </label><input type=numeric name=counter3 maxlength=20 value=";
  webPage += String(zaehlerstand3);
  webPage += "><br><br>"; 
  webPage += "<input type=submit value='SPEICHERN und Sensor Neustart'>";
  webPage += "</CENTER></body></html>";

  request->send(200, "text/html", webPage);
}
//------------------------------------------------------------------------------

void ModBus_setzen(AsyncWebServerRequest *request) {

  String webPage = "<!DOCTYPE HTML><html  lang=\"de\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  webPage += "<link rel='stylesheet' href='https://www.hoffmann-software.de/iot.css'>";
  webPage += "</head><body><CENTER>";
  webPage += "<div class=\"header\">";
  webPage += "<h2>Hoffmanns Sensor - Zählerstände</h2>";
  webPage += devicename +" aktuell mit " + WiFi.SSID() + " ("+ String(WiFi.RSSI()) +", C" + String(WiFi.channel())  +") und der IP: " + WiFi.localIP().toString() +" verbunden. <br><br>";
  webPage += "</div>"; 
  webPage += TOPNAVI;
  webPage += "Wählen Sie ihre Zähler und die dazugehörigen ModBus-Adressen.<br>";
  webPage += "<form action=/ method=POST>";
  webPage += "<table><tr><td width=60%>";
  String merke;
  for (uint8_t j = 0; j < 5; j++) {
  merke = eepromVar.modbus[j].device; merke.trim();
  webPage += "<label>ModBus Gerät " + String(j) + "</label>";
  webPage += "<select id=\"ModBus" + String(j) + "\" name=\"modbus" + String(j) + "\">";
  webPage += "<option value=''";
  if (merke=="") {webPage += " selected";}
  webPage += ">nicht installiert</option>";
  webPage += "<option value='SDM72D'";
   if (merke=="SDM72D") {webPage += " selected";};
  webPage += ">SDM72D</option>";
  webPage += "<option value='SDM630'";
  if (merke=="SDM630") {webPage += " selected";};
  webPage += ">SDM630</option>";
  webPage += "<option value='SDMXXX'";
  if (merke=="SDMXXX") {webPage += " selected";};
  webPage += ">SDMXXX</option> "; 
  webPage += "</select>";
  webPage += "</td><td width=40%>";
  webPage += "<label>ModBus Adresse</label><input type=numeric name='ADDR"+ String(j) +"' maxlength=3 value=" + String(eepromVar.modbus[j].addr) + ">";
  webPage += "</td></tr><tr><td>"; 
  } 
  webPage += "<input type=submit value='SPEICHERN'>";
  webPage += "</CENTER></body></html>";

  request->send(200, "text/html", webPage);
}
      

void WiFianmelden(AsyncWebServerRequest *request) {
  String webPage = "<!DOCTYPE HTML><html  lang=\"de\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  webPage += "<link rel='stylesheet' href='https://www.hoffmann-software.de/iot.css'>";
  webPage += "</head><body><CENTER>";
  webPage += "<div class=\"header\">";
  webPage += "<h2>Hoffmanns Multi-Sensor - WiFi Einstellungen</h2>";
  webPage += devicename +" aktuell mit+ " + WiFi.SSID() + " (" + String(WiFi.RSSI()) +", C" + String(WiFi.channel())  +") und der IP: " + WiFi.localIP().toString() +" verbunden. <br><br>";
  webPage += "</div>"; 
  webPage += TOPNAVI;
  webPage += "<br>Wollen Sie den Sensor in einem neuem Wifi anmelden?<br><br>";
  webPage += "<form action=/ method=POST>";
  webPage += "<label>Hostname</label><input fontsize=18 type=text name=devicename placeholder='devicename'><br>";
  webPage += "<label>SSID</label><input type=text name=wifiname placeholder='wifiname'><br>";
  webPage += "<label>Passwort</label><input type=password name=password placeholder='Password'><br><br>";
  webPage += "<input type=submit value='Anmelden'>";
 webPage +=  "</CENTER></body></html>";

  request->send(200, "text/html", webPage);
}
//------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
uint64_t HexWandel(String HexTemp) {
   uint64_t Multi = 1;
   uint64_t decwert = 0;
   int anzahl = HexTemp.length();
//   server.sendContent("HEX===>"+HexTemp+"==>"+ anzahl+"==>"+rxID+"\n");    
   for (int ii = anzahl-1; ii >= 0; ii = ii - 1) {
      if ((HexTemp[ii]=='a') || (HexTemp[ii]=='A')) {
           decwert += 10*Multi;Multi=Multi*16;              
      } else {
         if (HexTemp[ii]=='b') {
           decwert += 11*Multi;Multi=Multi*16;              
         } else {
            if (HexTemp[ii]=='c') {
              decwert += 12*Multi;Multi=Multi*16;              
            } else {
              if (HexTemp[ii]=='d') {
                 decwert += 13*Multi;Multi=Multi*16;              
              } else {
                  if (HexTemp[ii]=='e') {
                    decwert += 14*Multi;Multi=Multi*16;              
                  } else {
                    if (HexTemp[ii]=='f') {
                      decwert += 15*Multi;Multi=Multi*16;              
                    } else {
//                      server.sendContent("Wandel===>"+HexTemp+"==>"+String(ii)+"=>"+ HexTemp[ii]+"==>" + char(HexTemp[ii])+"==>" + String(int(HexTemp[ii]))+"==>"+String(decwert)+"\n");                          
                      rxID=HexTemp[ii];
                      decwert += rxID.toInt()*Multi;Multi=Multi*16;              
                    }
                  }                       
              }
            }    
         }
       }
   } 
   return(decwert);
}

void outputRead(AsyncWebServerRequest *request) {
  byte temp; int groesse=0;lesezeile = 0;leseindex=0;String Teststring = "";
  String output = "ID="; boolean Anfang = true; 
  for (int i=0; i<= 45; i++) {smlMessage[i] = "";}

  uint32_t startTime = millis();  
  
  while ((Serial.available() > 0) && (lesezeile<20)) {
    incomingByte = Serial.read();        
    if (incomingByte < 0x10) {
      smlMessage[lesezeile] += "0";
    }  
    smlMessage[lesezeile] += String(incomingByte, HEX);          
    if (incomingByte == 0x01) {
        incomingByte = Serial.read();        
        if (incomingByte == 0x77) {
//           server.sendContent(smlMessage[lesezeile]+"\n");                     
           lesezeile=lesezeile+1;
        }
        if (incomingByte == 0x76) {
//           server.sendContent(smlMessage[lesezeile]+"\n");                     
           lesezeile=lesezeile+1;
        }      
        if (incomingByte == 0xb1) {
//           server.sendContent(smlMessage[lesezeile]+"\n");                     
           lesezeile=lesezeile+1;
        }                 

        if (incomingByte < 0x10) {
           smlMessage[lesezeile] += "0";
        }  
        smlMessage[lesezeile] += String(incomingByte, HEX);          
    }
    Serial.write(incomingByte);
    if (millis() - startTime > m_timeout) break;    
  }
  String webpage = "";
  for (int i = 0; i < lesezeile; i = i + 1) { webpage += smlMessage[i]; webpage += "\n";}
  request->send(200, "text/plane", webpage);
}

void parseValues() {
  uint64_t merk64u=0ull;
  uint64_t merk64l=0ull;  
  std::string mystring;
  std::stringstream mystream;
  if (rxID == "0100000009ff") {
    rxID = "";int gerade=0; 
    for (int ii = 26; ii < 46; ii = ii + 1) {if (gerade>1) {rxID += "-";gerade=1;} else { gerade+=1;} rxID += smlMessage[leseindex][ii];}          
    ServerID = String(rxID);
    rxID = "";
    for (int ii = 38; ii < 46; ii = ii + 1) {rxID += smlMessage[leseindex][ii];}              
//    server.sendContent("Serie==>"+rxID+"\n");    
    SerienNr = String(HexWandel(rxID));
  }
  if ((rxID == "0100010800ff") || (rxID == "0200010800ff") || (rxID == "0200020800ff")) {
    rxID = "";
    for (int ii = 40; ii < 52; ii = ii + 1) {rxID += smlMessage[leseindex][ii];}          
//    server.sendContent("Bezug==>"+rxID+"\n");    
    merk64u = HexWandel(rxID);mystream.clear();
    mystream << merk64u;
    mystring = mystream.str();
    merk64l = consSum.toInt();   
    consSum = strcpy(new char[mystring.length() + 1], mystring.c_str());
    if ((consSum.toInt() != 0) and (merk64l != 0) and (consSum.toInt()> (merk64l*100))) {consSum = String(merk64l);}
  }           
  if (rxID == "0100020800ff") {
    rxID = "";
    for (int ii = 40; ii < 52; ii = ii + 1) {rxID += smlMessage[leseindex][ii];}          
    merk64u = HexWandel(rxID);mystream.clear();
    mystream << merk64u;
    mystring = mystream.str(); 
    merk64l = lieferSum.toInt();  
    lieferSum = strcpy(new char[mystring.length() + 1], mystring.c_str());
    if ((lieferSum.toInt() != 0) and (merk64l != 0) and (lieferSum.toInt()> (merk64l*100))) {lieferSum = String(merk64l);}    
//    lieferSum = String(HexWandel(rxID));      
  }
  if (rxID == "0100010801ff") {
    rxID = "";
    for (int ii = 34; ii < 46; ii = ii + 1) {rxID += smlMessage[leseindex][ii];}  
//    server.sendContent("BezugT1==>"+rxID+"\n");                
    merk64u = HexWandel(rxID);mystream.clear();
    mystream << merk64u;
    mystring = mystream.str();  
    merk64l = BezugT1.toInt(); 
    BezugT1 = strcpy(new char[mystring.length() + 1], mystring.c_str());
    if ((BezugT1.toInt() != 0) and (merk64l != 0) and (BezugT1.toInt()> (merk64l*100))) {BezugT1 = String(merk64l);}      
//    BezugT1 = String(HexWandel(rxID));
  }
  if (rxID == "0100010802ff") {
    rxID = "";
    for (int ii = 34; ii < 46; ii = ii + 1) {rxID += smlMessage[leseindex][ii];}          
//    server.sendContent("BezugT2==>"+rxID+"\n");
    merk64u = HexWandel(rxID);mystream.clear();
    mystream << merk64u;
    mystring = mystream.str();   
    BezugT2 = strcpy(new char[mystring.length() + 1], mystring.c_str());
//    BezugT2 = String(HexWandel(rxID));
  }
/*  if (rxID == "0100100700ff") {
    for (int ii = 32; ii < 46; ii = ii + 1) {rxID += smlMessage[leseindex][ii];}          
    merk64u = HexWandel(rxID);
    mystream << merk64u;
    mystring = mystream.str();    
    powerL1 = strcpy(new char[mystring.length() + 1], mystring.c_str());;

//    powerL1 = String(HexWandel(rxID));
  }  */
  if (rxID == "0100240700ff") {   //1-0:56.7.0*255
    rxID = "";
    for (int ii = 34; ii < 46; ii = ii + 1) {rxID += smlMessage[leseindex][ii];}          
    merk64u = HexWandel(rxID);mystream.clear();
    mystream << merk64u;
    mystring = mystream.str();   
    powerL2 = strcpy(new char[mystring.length() + 1], mystring.c_str());
//    powerL2 = String(HexWandel(rxID));
  }             
  if (rxID == "0100380700ff") {  //1-0:76.7.0*255
    rxID = "";
    for (int ii = 34; ii < 46; ii = ii + 1) {rxID += smlMessage[leseindex][ii];}          
    merk64u = HexWandel(rxID);mystream.clear();
    mystream << merk64u;
    mystring = mystream.str();   
    powerL3 = strcpy(new char[mystring.length() + 1], mystring.c_str());

//    powerL3 = String(HexWandel(rxID));
  }    
  if (rxID == "0100100700ff") {  //1-0:16.7.0*255
    rxID = "";
    for (int ii = 34; ii < 46; ii = ii + 1) {rxID += smlMessage[leseindex][ii];}      
    merk64u = HexWandel(rxID);mystream.clear();
    mystream << merk64u;
    mystring = mystream.str();   
    powerSum = strcpy(new char[mystring.length() + 1], mystring.c_str());
  } 
  if (rxID == "0100600505ff") {  //1-0:96.5.5*255
    mtrState = String(rxBuffer);
  }
}

void uploadValues(AsyncWebServerRequest *request) {
  byte temp; int groesse=0;lesezeile = 1;leseindex=0;String Teststring = "";
  String output = "ID="; boolean Anfang = true; 
  for (int i=0; i<= 45; i++) {smlMessage[i] = "";}
  uint32_t startTime = millis();
  
  while ((Serial.available() > 0) && (lesezeile<40)) {
    incomingByte = Serial.read();        
    if (incomingByte < 0x10) {
      smlMessage[lesezeile] += "0";
    }  
    smlMessage[lesezeile] += String(incomingByte, HEX);          
    if (incomingByte == 0x01) {
        incomingByte = Serial.read();        
        if (incomingByte == 0x77) {
           //server.sendContent(smlMessage[lesezeile]+"\n");                     
           lesezeile=lesezeile+1;
        }
        if (incomingByte < 0x10) {
           smlMessage[lesezeile] += "0";
        }  
        smlMessage[lesezeile] += String(incomingByte, HEX);          
    }
    Serial.write(incomingByte);
    if (sizeof(smlMessage[lesezeile])>300) {break;}
    if (millis() - startTime > m_timeout) break;
  }
  
  for (int i = 0; i < lesezeile; i = i + 1) {
//       server.sendContent(smlMessage[i]);rxID = "";
       for (int ii = 0; ii < 4; ii = ii + 1) {rxID += smlMessage[i][ii];}
 //      server.sendContent("==>"+rxID+"\n");
       if (rxID="7707") {
          rxID = "";
          for (int ii = 4; ii < 16; ii = ii + 1) {rxID += smlMessage[i][ii];}
//          server.sendContent("====>"+rxID+"\n");
          leseindex=i;  parseValues();
       }
  }
  String webPage; 
  StaticJsonDocument<256> doc;

  doc["device"] = "Easymeter/Q3M";
  doc["sensor"] = WiFi.hostname();
  doc["ServerID"] = ServerID;
  doc["SerienNr"] = SerienNr;
  if (consSum.toInt()>0) {doc["Bezug"] = consSum;} else {doc["Bezug"] = "Nan";}
  if (lieferSum.toInt()>0) {doc["Liefer"] = lieferSum;} else {doc["Liefer"] = "Nan";}  
  if (powerSum.toInt()>0) {doc["Power"] = powerSum;} else {doc["Power"] = "Nan";}    
  if (BezugT1.toInt()>0) {doc["BezugT1"] = BezugT1;} else {doc["BezugT1"] = "Nan";} 
  doc["BezugT2"] = BezugT2;  
  powerL1 = String(powerSum.toInt()-(powerL2.toInt()+powerL3.toInt())); 
  doc["L1"] = powerL1;
  doc["L2"] = powerL2;
  doc["L3"] = powerL3;
  serializeJson(doc, webPage);
  request->send(200, "text/plane", webPage);
}


void LieferungValues(AsyncWebServerRequest *request) {
  byte temp; int groesse=0;lesezeile = 1;leseindex=0;String Teststring = "";
  String output = "ID="; boolean Anfang = true; 
  for (int i=0; i<= 45; i++) {smlMessage[i] = "";}
  uint32_t startTime = millis();  
  while ((Serial.available() > 0) && (lesezeile<20)) {
    incomingByte = Serial.read();        
    if (incomingByte < 0x10) {
      smlMessage[lesezeile] += "0";
    }  
    smlMessage[lesezeile] += String(incomingByte, HEX);          
    if (incomingByte == 0x01) {
        incomingByte = Serial.read();        
        if (incomingByte == 0x77) {
           //server.sendContent(smlMessage[lesezeile]+"\n");                     
           lesezeile=lesezeile+1;
        }
        if (incomingByte < 0x10) {
           smlMessage[lesezeile] += "0";
        }  
        smlMessage[lesezeile] += String(incomingByte, HEX);          
    }
    Serial.write(incomingByte);
    if (millis() - startTime > m_timeout) break;        
  }
  
  for (int i = 0; i < lesezeile; i = i + 1) {
//       server.sendContent(smlMessage[i]);rxID = "";
       for (int ii = 0; ii < 4; ii = ii + 1) {rxID += smlMessage[i][ii];}
 //      server.sendContent("==>"+rxID+"\n");
       if (rxID="7707") {
          rxID = "";
          for (int ii = 4; ii < 16; ii = ii + 1) {rxID += smlMessage[i][ii];}
//          server.sendContent("====>"+rxID+"\n");
          leseindex=i;  parseValues();
       }
  }
  String webpage ="";
  webpage += lieferSum;
  request->send(200, "text/plane", webpage);  
}

void BezugValues(AsyncWebServerRequest *request) {
//  Serial.begin(9600, SERIAL_7E1);
//  Serial.begin(9600);
  byte temp; int groesse=0;lesezeile = 1;leseindex=0;String Teststring = "";
  String output = "ID="; boolean Anfang = true; 
  for (int i=0; i<= 45; i++) {smlMessage[i] = "";}
  uint32_t startTime = millis();  
  while ((Serial.available() > 0) && (lesezeile<20)) {
    incomingByte = Serial.read();        
    if (incomingByte < 0x10) {
      smlMessage[lesezeile] += "0";
    }  
    smlMessage[lesezeile] += String(incomingByte, HEX);          
    if (incomingByte == 0x01) {
        incomingByte = Serial.read();        
        if (incomingByte == 0x77) {
           //server.sendContent(smlMessage[lesezeile]+"\n");                     
           lesezeile=lesezeile+1;
        }
        if (incomingByte < 0x10) {
           smlMessage[lesezeile] += "0";
        }  
        smlMessage[lesezeile] += String(incomingByte, HEX);          
    }
    if (millis() - startTime > m_timeout) break;        
    Serial.write(incomingByte);
  }
  
  for (int i = 0; i < lesezeile; i = i + 1) {
//       server.sendContent(smlMessage[i]);rxID = "";
       for (int ii = 0; ii < 4; ii = ii + 1) {rxID += smlMessage[i][ii];}
 //      server.sendContent("==>"+rxID+"\n");
       if (rxID="7707") {
          rxID = "";
          for (int ii = 4; ii < 16; ii = ii + 1) {rxID += smlMessage[i][ii];}
//          server.sendContent("====>"+rxID+"\n");
          leseindex=i;  parseValues();
       }
  }
  String webpage ="";
  webpage += consSum;
  request->send(200, "text/plane", webpage);
}

void outputValues(AsyncWebServerRequest *request) {
//  Serial.begin(9600, SERIAL_7E1);
//  if (WiFi.status() == WL_CONNECTED) {
  byte temp; int groesse=0;lesezeile = 1;leseindex=0;String Teststring = "";
  String output = "ID="; boolean Anfang = true; 
  for (int i=0; i<= 45; i++) {smlMessage[i] = "";}
  uint32_t startTime = millis();  
  while ((Serial.available() > 0) && (lesezeile<20)) {
    incomingByte = Serial.read();        
    if (incomingByte < 0x10) {
      smlMessage[lesezeile] += "0";
    }  
    smlMessage[lesezeile] += String(incomingByte, HEX);          
    if (incomingByte == 0x01) {
        incomingByte = Serial.read();        
        if (incomingByte == 0x77) {
           //server.sendContent(smlMessage[lesezeile]+"\n");                     
           lesezeile=lesezeile+1;
        }
        if (incomingByte < 0x10) {
           smlMessage[lesezeile] += "0";
        }  
        smlMessage[lesezeile] += String(incomingByte, HEX);          
    }
    if (millis() - startTime > m_timeout) break;        
    Serial.write(incomingByte);
  }
  
  for (int i = 0; i < lesezeile; i = i + 1) {
//       server.sendContent(smlMessage[i]);rxID = "";
       for (int ii = 0; ii < 4; ii = ii + 1) {rxID += smlMessage[i][ii];}
 //      server.sendContent("==>"+rxID+"\n");
       if (rxID="7707") {
          rxID = "";
          for (int ii = 4; ii < 16; ii = ii + 1) {rxID += smlMessage[i][ii];}
//          server.sendContent("====>"+rxID+"\n");
          leseindex=i;  parseValues();
       }
  }
  String webpage ="<html><body>";
  webpage +="Zähler    = Easymeter/Q3M <br>";
  webpage +="Sensor    = " +WiFi.hostname() + "<br>";
  webpage +="LocalIP   = ";
  webpage += WiFi.localIP().toString();
  webpage +=" <br>";
  webpage +="Server-ID = " + ServerID + "<br>";
  webpage +="Serien-Nr = " + SerienNr+ "<br>";
  webpage +="Bezug     = " + consSum+ "<br>";
  webpage +="Lieferung = "+ lieferSum+ "<br>"; 
  powerL1 = String(powerSum.toInt()-(powerL2.toInt()+powerL3.toInt())); 
  webpage +="Leitung 1 = "+ powerL1+ "<br>";  
  webpage +="Leitung 2 = "+ powerL2+ "<br>";
  webpage +="Leitung 3 = "+ powerL3+ "<br>";      
  webpage +="Leistung  = "+ powerSum+ "<br>";     
  webpage +="Bezug Tarif 1 = " + BezugT1 + "<br>";  
  webpage +="Bezug Tarif 2 = " + BezugT2 + "<br>";  
/*  webpage +="Stundenstatistik: \n";
  for (int ii = 1; ii < 24; ii = ii + 1) {
    if (Stundenverbrauch[ii]>Stundenverbrauch[ii-1]) {
      webpage += String(ii) + ". " + String(Stundenverbrauch[ii]-Stundenverbrauch[ii-1]) + "\n"; 
    } else {webpage += String(ii) + ". 0\n"; }
  } */ 
  webpage +="</body></html>";
  request->send(200, "text/html", webpage);
}
//-------------------------EASYmeter ------------------------------------------------------------


void handleLogin(AsyncWebServerRequest *request) {                         // If a POST request is made to URI /login
  if ((request->hasArg("modbus0")) or (request->hasArg("modbus1")) or (request->hasArg("modbus2")) or (request->hasArg("modbus3")) or (request->hasArg("modbus4"))) {    
      for (uint8_t i = 0; i < 5; i++) {
        String merks = "modbus"+String(i);
        const char* merkc = merks.c_str();
        if (request->hasArg(merkc)) {
         String merke = request->arg(merkc); merke.trim();        
         merke.toCharArray(eepromVar.modbus[i].device,8); 
         merke = request->arg("ADDR"+String(i)); merke.trim();
         eepromVar.modbus[i].addr = merke.toInt();
        } else {
          String merke = "";
          merke.toCharArray(eepromVar.modbus[i].device,8);
          eepromVar.modbus[i].addr = 0;
        }
      }
      writeToMemory(wifi_Name, wifi_Pass, devicename, zaehlerstand, zaehlerstand2, zaehlerstand3);        
  }
  if (request->hasArg("counter")) {
    zaehlerstand = request->arg("counter").toInt();
    zaehlerstand2 = request->arg("counter2").toInt();
    zaehlerstand3 = request->arg("counter3").toInt();
    writeToMemory(wifi_Name, wifi_Pass, devicename, zaehlerstand, zaehlerstand2, zaehlerstand3);  
  } else {
  if ((request->hasArg("wifiname") and (request->hasArg("password"))) 
      and ((request->arg("wifiname") != NULL) and (request->arg("password") != NULL))) { // If the POST request doesn't have username and password data        
        wifi_Name = request->arg("wifiname");
        wifi_Name.trim();               
        wifi_Pass = request->arg("password");
        wifi_Pass.trim();   
        devicename = request->arg("devicename");
        wifi_Pass.trim();   
        Serial.print("SSIP:" + wifi_Name + "  Passwort: " + wifi_Pass);
        writeToMemory(wifi_Name, wifi_Pass, devicename, zaehlerstand, zaehlerstand2, zaehlerstand3);                 
  }
  Serial.print("kein Speichern ... SSIP:" + wifi_Name + "  Passwort: " + wifi_Pass);
  }
  indexrequest(request);
}

void ledOn() {
  digitalWrite(LED_BUILTIN, LOW);
}
//------------------------------------------------------------------------------
void ledOff() {
  digitalWrite(LED_BUILTIN, HIGH);
}
//------------------------------------------------------------------------------
void ledSwap() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
//------------------------------------------------------------------------------
void otaInit() {
  ArduinoOTA.setHostname(devicename.c_str());

  ArduinoOTA.onStart([]() {
    ledOn();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    ledSwap();
  });
  ArduinoOTA.onEnd([]() {
    ledOff();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    ledOff();
  });
  ArduinoOTA.begin();
}
//------------------------------------------------------------------------------

void serverInit() {
  
  server.on("/", HTTP_GET, indexrequest);  
  server.on("/bus", HTTP_GET, ModBus_setzen);
  server.on("/xml", HTTP_GET, xmlrequest);  
  server.on("/json", HTTP_GET, jsonrequest);
  server.on("/jsonz", HTTP_GET, jsonrequestz);  
  server.on("/direct", HTTP_GET, registerdirect);     // zwei Einzelwert möglich!!!  ? ADDR=1 -> Modbus Adresse (ADDR2) und REG=53  (REG2) ->0x0034 Achtung Dezimal übegeben! WERT=1 oder 2 zur direkt anzeige
  server.on("/tabelle", HTTP_GET, tabelleinbinden);
  server.on("/wifi", HTTP_GET, WiFianmelden);  
  server.on("/stand", HTTP_GET, Zaehler_setzen);
  server.on("/", HTTP_POST, handleLogin);  //Formular für die WiFi Anmeldung
  server.on("/jsoneasy", HTTP_GET,uploadValues);    //Ausgabe der Werte im JSON Format zum einlesen z.B. im Home Assistant
  server.on("/easy", HTTP_GET, outputValues);        //Browserausgabe der Zählerwerte
  server.on("/bezug",HTTP_GET, BezugValues);    //Ausgabe des Wertes im Text-Format zum einlesen z.B. im Home Assistant  
  server.on("/lieferung",HTTP_GET, LieferungValues);    //Ausgabe des Wertes im Text-Format zum einlesen z.B. im Home Assistant    
  server.on("/lesen",HTTP_GET, outputRead);     //Ausgabe der eingelesen Daten  
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404);
  });
  server.begin();
}
//------------------------------------------------------------------------------

static void wifiInit() {
  
  WiFi.persistent(true);                                                       // Do write new connections to FLASH
  WiFi.mode(WIFI_OFF);    
  WiFi.mode(WIFI_STA);
  if (readFromMemory()) {
    WiFi.setAutoConnect(false);
    WiFi.setAutoReconnect(true);  
    WiFi.begin(wifi_Name,wifi_Pass);
    int versuch = 0;  
    while( WiFi.status() != WL_CONNECTED ) {                                      //  Wait for WiFi connection
      ledSwap();
      delay(50);
      versuch++;
      if (versuch>120) {break;}                                                   //  Versuche 2 Minuten zu verbinden, gelingt dies nicht Raus
    } 
    WiFi.setSleep(false);
  }
  if (WiFi.status() != WL_CONNECTED) {  
  WiFi.begin(wifi_ssid, wifi_password);  
  int versuch = 0;
  while( WiFi.status() != WL_CONNECTED ) {                                      //  Wait for WiFi connection
    ledSwap();
    delay(200);
    versuch++;
    if (versuch>60) {break;}                                                   //  Versuche 2 Minuten zu verbinden, gelingt dies nicht Raus
  }

  if (WiFi.status() != WL_CONNECTED) {
  bool wpsSuccess = WiFi.beginWPSConfig();
  while( WiFi.status() != WL_CONNECTED ) {                                      //  Wait for WiFi connection
    ledSwap();
    delay(200);
    versuch++;
    if (versuch>60) {break;}                                                   //  Versuche 2 Minuten zu verbinden, gelingt dies nicht Raus
  }
  if(wpsSuccess) {
      // Well this means not always success :-/ in case of a timeout we have an empty ssid
      String newSSID = WiFi.SSID();
      if(newSSID.length() > 0) {
        // WPSConfig has already connected in STA mode successfully to the new station. 
        Serial.printf("WPS finished. Connected successfull to SSID '%s'\n", newSSID.c_str());
      } else {
        wpsSuccess = false;
      }
  }

  if (WiFi.status() != WL_CONNECTED) {
    ledOn();
    WiFi.mode(WIFI_AP);                                                           // Accept Point Mode      
    WiFi.config(ip, gateway, subnet);
    WiFi.softAP(wifiName1, wifiPass1);
     delay(1000);
  }
  }
  }
  if (MDNS.begin(devicename)) {  //Start mDNS
      Serial.println("MDNS started");
      }
      
}
//------------------------------------------------------------------------------

void sdmRead() {
  
  float tmpval = NAN;String merke="";

  if (einzel_addr>0) {
      tmpval = sdm.readVal(einzel_register, einzel_addr);
      if (!(isnan(tmpval))) einzel_abruf = tmpval;   
  } 
  if (einzel2_addr>0) {
      tmpval = sdm.readVal(einzel2_register, einzel2_addr);
      if (!(isnan(tmpval))) einzel2_abruf = tmpval;   
  }   
  for (uint8_t j = 0; j < 5; j++) {
    merke = eepromVar.modbus[j].device; merke.trim();
    if (merke=="SDM72D") {
      for (uint8_t i = 0; i < NBREG; i++) {
        if (eepromVar.modbus[j].addr>0) {
          tmpval = sdm.readVal(sdmarr[i].regarr, eepromVar.modbus[j].addr);
          if (!(isnan(tmpval))) sdmarr[i].regvalarr[j] = tmpval;
        }
      }
    } else {
      if (merke=="SDM630") {
        for (uint8_t i = 0; i < NBREG2; i++) { 
          if (eepromVar.modbus[j].addr>0) {                       
            tmpval = sdm.readVal(sdmarr2[i].regarr, eepromVar.modbus[j].addr);
            if (!(isnan(tmpval))) sdmarr2[i].regvalarr[j] = tmpval;
          }
        }
      } else {
        if (merke=="SDMXXX") {
          for (uint8_t i = 0; i < NBREG1; i++) {   
            if (eepromVar.modbus[j].addr>0) {                     
               tmpval = sdm.readVal(sdmarr1[i].regarr,eepromVar.modbus[j].addr);
               if (!(isnan(tmpval))) sdmarr1[i].regvalarr[j] = tmpval;
            }               
          }
        }
      }
    }
    yield();
  }

//    if (!(isnan(tmpval)))
//     sdmarr[i].regvalarr = 0.00;
//   else
//      sdmarr[i].regvalarr[adresse] = tmpval;
}
//------------------------------------------------------------------------------

void IRAM_ATTR Ext_INT1_ISR()                        // Interrupt bei jedem Anstieg also schlissen des Zaehlers
{
  zaehlerstand++;
}
//------------------------------------------------------------------------------

void IRAM_ATTR Ext_INT1_ISR2()                        // Interrupt bei jedem Anstieg also schlissen des Zaehlers
{
  zaehlerstand2++;
}
//------------------------------------------------------------------------------

void IRAM_ATTR Ext_INT1_ISR3()                        // Interrupt bei jedem Anstieg also schlissen des Zaehlers
{
  zaehlerstand3++;
}
//------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  ledOn();
  lastresetreason = ESP.getResetReason();
  wifiInit();
  otaInit();
  serverInit();
  sdm.begin();
  attachInterrupt(D4, Ext_INT1_ISR, RISING);        // Interrupt tricker event RISING	FALLING HIGH LOW CHANGE
  attachInterrupt(D7, Ext_INT1_ISR2, RISING);        // Interrupt tricker event RISING	FALLING HIGH LOW CHANGE  
  attachInterrupt(D8, Ext_INT1_ISR3, RISING);        // Interrupt tricker event RISING	FALLING HIGH LOW CHANGE    
  readtime = millis();
  ledOff();
}
//------------------------------------------------------------------------------
void loop() {
  ArduinoOTA.handle();
  if (millis() - readtime >= READSDMEVERY_C) {
    sdmRead();
    readtime = millis();
  }

  yield();
}
