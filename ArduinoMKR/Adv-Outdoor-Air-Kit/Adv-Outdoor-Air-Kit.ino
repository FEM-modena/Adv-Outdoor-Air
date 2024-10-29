/* 
  Advanced Outdoor Air Kit
  Future Education Modena 2024-2025

  Monitora:   
  - Particolato PM10-PM2.5 (DFRobot SEN0460)
  - Ozono (DFRobot SEN0321)
  - Ossido di Carbonio (DFRobot SEN0466)
  - Ammoniaca (DFRobot SEN0469)
  - Biossido di Azoto (DFRobot SEN0471)
  - Metano (DFRobot SEN0129)
*/

///// NON SPOSTARE QUESTE DEFINIZIONI  /////
//*******************************************
#define SECRET_SSID "FEM_WiFi"
#define SECRET_PASS "0h4orXc@yS3do"
#define CHIAVE_CLOUD "FEMGreenAirExplorer_serra"

char dboard_server[] = "demo.thingsboard.io"; // Indirizzo IP/Internet del Dashboard Server
int dboard_port = 80;                         // Porta TCP del server

//Variabili
float temp_aria = 0.0;
float umid_aria = 0.0;
uint16_t pm10 = 0;
uint16_t pm2_5 = 0;
uint16_t ozono = 0;
uint16_t oss_carbonio = 0;
uint16_t ammoniaca = 0;
uint16_t bioss_azoto = 0;
int metano = 0;

boolean pm_ON = true;
boolean ozono_ON = true;
boolean oss_carb_ON = true;
boolean ammon_ON = true;
boolean bioss_az_ON = true;

//Collegare una resistenza da 1K o il Grove Led Rosso
#define PIN_LED1 5
//Collegare RESET a questo pin con un jumper
#define PIN_RESET 4

//Collegamento alla funzione del led
void accendi_LED_per(byte volte);

//Collegamento alla piattaforma GL-Blocks
#include "GL-Blocks-WiFi.h"
#include "GL-Blocks-Dashboard-AOA.h"
//*******************************************
///// NON SPOSTARE QUESTE DEFINIZIONI  /////

#define CICLI_ATT_SENS 5

#include <Wire.h>  

//Libreria ENV Shield di Arduino
//Installare da "Gestione Librerie"
#include <Arduino_MKRENV.h>

//Libreria per sensore particolato DFRobot SEN0460
//wiki.dfrobot.com/Gravity_PM2.5_Air_Quality_Sensor_SKU_SEN0460
#include "DFRobot_AirQualitySensor.h"
#define PM_IICADDRESS    0x19
DFRobot_AirQualitySensor particle(&Wire ,PM_IICADDRESS);

//Libreria per sensore ozono DFRobot SEN0321
#include "DFRobot_OzoneSensor.h"
#define OZONE_COLLECT_NUMBER   20   // collect number, the collection range is 1-100
/*   IIC slave Address, The default is ADDRESS_3
       ADDRESS_0               0x70      // iic device address
       ADDRESS_1               0x71
       ADDRESS_2               0x72
       ADDRESS_3               0x73
*/
#define OZONE_IIC_ADDRESS   0x73
DFRobot_OzoneSensor ozone;

//Libreria unica per i 3 sensori GAS
#include "DFRobot_MultiGasSensor.h"
//Definizioni per Sensore CO SEN0466
#define CO_IIC_ADDRESS    0x74
DFRobot_GAS_I2C co_sens(&Wire, CO_IIC_ADDRESS);
//Definizioni per Sensore NH3 SEN0469
#define NH3_IIC_ADDRESS   0x75
DFRobot_GAS_I2C nh3_sens(&Wire, NH3_IIC_ADDRESS);
//Definizioni per Sensore NO2 SEN0471
#define NO2_IIC_ADDRESS   0x76
DFRobot_GAS_I2C no2_sens(&Wire, NO2_IIC_ADDRESS);

//Metano: lettura analogica
#define CONC_RIF_METANO 5000
#define PIN_METANO A0

/**
 * Preperazione di Arduino: setup() 
 * Eseguito una sola volta.
 */
void setup() {

  //Configura il pin per il reset
  digitalWrite(PIN_RESET, HIGH);
  delay(100);
  pinMode(PIN_RESET, OUTPUT);

  //Attiva il Serial Monitor
  Serial.begin(9600);  
  delay(2000); //Tempo per aprire il serial monitor...
  Serial.println("FEM - Green Air Explorer Kit");

  pinMode(PIN_LED1, OUTPUT);
  digitalWrite(PIN_LED1, LOW);
  
  accendi_LED_per(1); //Lampeggia 1 volta

  Wire.begin(); //Inzializza I2C per l'ENV shield 
  accendi_LED_per(2); //Lampeggia 2 volte

  ///// ATTIVAZIONE DEI SENSORI /////
 
  //Si connette L'ENV Shield
  if (ENV.begin()) Serial.println ("ENV Shield attivato");
  else  {
    Serial.println("Errore durante l'avvio del ENV Shield. STOP.");
    while (1);
  }   

  //Attivazione sensore particolato
  if (particle.begin()) Serial.println ("Sensore particolato attivo.");
  else {
    Serial.println("Sensore particolato SEN0460 non trovato");
    pm_ON = false;
  }
  delay(500);

  //Attivazione sensore Ozono
  if (ozone.begin(OZONE_IIC_ADDRESS)) {
    ozone.setModes(MEASURE_MODE_PASSIVE);
    Serial.println ("Sensore Ozono SEN0321 attivo");
  }
  else {
    Serial.println("Sensore Ozono SEN0321 non trovato");
    ozono_ON = false;
  }
  delay(500);  

  //Attivazione sensore CO
  if (co_sens.begin()) {
    co_sens.changeAcquireMode(co_sens.PASSIVITY);
    delay(500);
    Serial.println(co_sens.queryGasType());
    co_sens.setTempCompensation(co_sens.OFF);
    Serial.println ("Sensore CO SEN0466 attivo");
  }
  else {
    Serial.println("Sensore CO SEN0466 non trovato");
    oss_carb_ON = false;
  }
  delay(500);

  //Attivazione sensore NH3
  if (nh3_sens.begin()) {
    nh3_sens.changeAcquireMode(nh3_sens.PASSIVITY);
    delay(500);
    Serial.println(nh3_sens.queryGasType());
    nh3_sens.setTempCompensation(nh3_sens.OFF);
    Serial.println ("Sensore NH3 SEN0469 attivo.");  
  }
  else {
    Serial.println("Sensore NH3 SEN0469 non trovato");
    ammon_ON = false;
  }
  delay(500);

  //Attivazione sensore NO2
  if (no2_sens.begin()) {
    no2_sens.changeAcquireMode(no2_sens.PASSIVITY);
    delay(500);
    Serial.println(no2_sens.queryGasType());
    no2_sens.setTempCompensation(no2_sens.OFF);
    Serial.println ("Sensore NO2 SEN0471 attivo.");  
  }
  else {
    Serial.println("Sensore NO2 SEN0471 non trovato");
    bioss_az_ON = false;
  }
  delay(500);

  accendi_LED_per(3); //Lampeggia 3 volte


  // Connessione al WiFi NON BLOCCANTE: vedi il file GL-Blocks-WiFi.h
  if (Connetti_WIFI()) accendi_LED_per(4); //Lampeggia 4 volte: PRONTI   
  else Serial.println("WiFi non disponibile in avvio");
}

/**
 * Ciclo delle operazioni da eseguire sempre
 */
void loop() {    

  accendi_LED_per(3);

  //Temperatura dall'ENV Shield
  temp_aria = ENV.readTemperature();

  //Umidità aria dall'ENV Shield
  umid_aria = ENV.readHumidity();

  //Lettura particolato
  if (pm_ON) {
    Serial.print("Lettura Particolato...");
    pm2_5= particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD);    
    pm10 = particle.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD);
    delay(100);
  }
  
  //Lettura Ozono
  if (ozono_ON) {
    Serial.print("Lettura Ozono...");
    ozono = ozone.readOzoneData(OZONE_COLLECT_NUMBER);
    delay(100);
  }

  //Lettura CO
  if (oss_carb_ON) {
    Serial.print("Lettura CO...");
    oss_carbonio = co_sens.readGasConcentrationPPM();
    delay(100);
  }

  if (ammon_ON) {
    Serial.print("Lettura NH3...");
    ammoniaca = nh3_sens.readGasConcentrationPPM();
    delay(100);
  }

  if (bioss_az_ON) {
    Serial.print("Lettura NO2...");
    bioss_azoto = no2_sens.readGasConcentrationPPM();
    delay(100);
  }

  int val=analogRead(PIN_METANO);
  //TODO: aggiustamento al valore reale
  //Richiede calilbrazione!
  metano = val * CONC_RIF_METANO / 1024;

  accendi_LED_per(2); //Lampeggia il led per 2 volte
      
  mostra_valori_serial_monitor();

  if (Connetti_WIFI()) {
    delay(1000);
    Trasmetti_Dati_Cloud();
  }
  else {
    Serial.println("Dati non inviati alla dashboard: no WiFi");
  }  
  //30 sec tra un ciclo e il prossimo
  delay(30000); 
}

/**
 * Lampeggia il LED sul pin PIN_LED1 per un numero di volte
 */
void accendi_LED_per(byte volte) 
{
  for (byte i=0; i<volte; i++) {
    digitalWrite(PIN_LED1, HIGH);
    delay(200);
    digitalWrite(PIN_LED1, LOW);
    delay(200);
  }
}

/**
 * Scrive i valori dei sensori sul serial Monitor (Serial)
 */
void mostra_valori_serial_monitor()
{
  Serial.println();    
  Serial.print("Temp. aria = ");
  Serial.print(temp_aria);
  Serial.println(" °C");

  Serial.print("Umid. aria = ");
  Serial.print(umid_aria);
  Serial.println(" %");
  
  Serial.print("Concentrazione PM2.5 = ");
  Serial.print(pm2_5);
  Serial.println(" ug/m3");
  Serial.print("Concentrazione PM10 = ");
  Serial.print(pm10);
  Serial.println(" ug/m3");

  Serial.print("Concentrazione Ozono =  ");
  Serial.print(ozono);
  Serial.println(" PPB.");

  Serial.print("Concentrazione CO = ");
  Serial.print(oss_carbonio);
  Serial.println(" %vol");

  Serial.print("Concentrazione NH3 = ");
  Serial.print(ammoniaca);
  Serial.println(" %vol");

  Serial.print("Concentrazione NO2 = ");
  Serial.print(bioss_azoto);
  Serial.println(" %vol");

  Serial.print("Concentrazione Metano = ");
  Serial.print(metano);
  Serial.println(" PPM");

}