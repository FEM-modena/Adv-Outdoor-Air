/* 
  Advanced Outdoor Air Kit
  Future Education Modena 2024-2025

  Monitora:   
  - Particolato PM10-PM2.5 (DFRobot SEN0460)
  - Ozono (DFRobot SEN0321)
  - Ossido di Carbonio (DFRobot SEN0466
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
int pm10 = 0;
int pm2_5 = 0;
int ozono = 0;
int oss_carbonio = 0;
int ammoniaca = 0;
int bioss_azoto = 0;
int metano = 0;

boolean pm_ON = true;
boolean ozono_ON = true;
boolean oss_carb_ON = true;
boolean ammon_ON = true;
boolean bioss_az_ON = true;
boolean metano_ON = true;

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
#define OZCOLLECT_NUMBER   20   // collect number, the collection range is 1-100
/*   IIC slave Address, The default is ADDRESS_3
       ADDRESS_0               0x70      // iic device address
       ADDRESS_1               0x71
       ADDRESS_2               0x72
       ADDRESS_3               0x73
*/
#define Ozone_IICAddress OZONE_ADDRESS_3
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
  if (!ENV.begin()) {
    Serial.println("Errore durante l'avvio del MKR Shield");
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
  if (ozone.begin(Ozone_IICAddress)) {
    ozone.SetModes(MEASURE_MODE_PASSIVE);
    Serial.println ("Sensore Ozono attivo.");
  }
  else {
    Serial.println("Sensore Ozono SEN0321 non trovato");
    ozono_ON = false;
  }  

  //Attivazione sensore CO
  if (co_sens.begin()) {
    co_sens.changeAcquireMode(co_sens.PASSIVITY);
    delay(500);
    co_sens.setTempCompensation(co_sens.OFF);  
  }
  else {
    Serial.println("Sensore CO SEN0466 non trovato");
    oss_carb_ON = false;
  }

    

  accendi_LED_per(3); //Lampeggia 3 volte

  // Connessione al WiFi: vedi il file GL-Blocks-WiFi.h
  Connetti_WIFI();  
  
  accendi_LED_per(4); //Lampeggia 4 volte: PRONTI   

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

  //Luminosità dall'ENV Shield
  luminosita = ENV.readIlluminance();

  //Lettura sensore capacitivo umidità terreno
  umid_terreno1 = leggi_sens_umidita(PIN_UMIDITA_1);
  umid_terreno2 = leggi_sens_umidita(PIN_UMIDITA_2);
  umid_terreno3 = leggi_sens_umidita(PIN_UMIDITA_3);

  //Misura dell'Anidride carbonica
  float result[3] = {0};
  //Se il risultato è disponibile, lo legge
  if(scd30.isAvailable()) {
    Serial.print("Misura CO2 disponibile");        
    scd30.getCarbonDioxideConcentration(result);
    anid_carbonica = result[0];
    last_ppmCO2 = anid_carbonica;
  }
  else {
    Serial.print("Misura CO2 non ancora disponibile: non aggiornato");        
    anid_carbonica = last_ppmCO2;        
  }    

  //Misura del particolato
  unsigned long durata = pulseIn(PIN_SENSORE_PARTICOLATO, LOW);
  durataImpulsoLow = durataImpulsoLow + durata;

  if ((millis() - ultimaLetturaDust) > INTERVALLO_PARTICOLATO) {
    float ratio = durataImpulsoLow/(INTERVALLO_PARTICOLATO * 10.0);
    particolato = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;

    durataImpulsoLow = 0;
    ultimaLetturaDust = millis();
  } 

  //Misura dell'Air Quality
  aq_tend = sensore_aq.slope();
  switch (aq_tend) {
    case 0: 
      aq_stato = "ALLARME";
      break;
    case 1:
      aq_stato = "INQUINATO";
      break;
    case 2:
      aq_stato = "BASSO INQUINAMENTO";
      break;
    case 3:  
      aq_stato = "QUALITA' BUONA";
  }
  aq_valore = sensore_aq.getValue();
  
  accendi_LED_per(2); //Lampeggia il led per 2 volte
      
  mostra_valori_serial_monitor();

  //Verifica se si è ancora connessi al WiFi
  Connetti_WIFI();
  
  delay(1000); //Attende 1 secondo

  //Invia i dati alla dashboard
  Trasmetti_Dati_Cloud();    

  //30 sec tra un ciclo e il prossimo
  delay(30000); 
}

/**
 * Procedura di lettura umidità terreno
 */
int leggi_sens_umidita(int pin)
{
  delay(10); //Breve ritardo per letture analogiche consecutive  
  long va = analogRead(pin);
  int umid = (int)map(va, 520, 760, 100, 0); //Tarato sui sensori capacitivi Grove
  umid = constrain(umid, 0, 100);
  return umid;
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

  Serial.print("Umid. terr. s.1 = ");
  Serial.print(umid_terreno1);
  Serial.println(" %");

  Serial.print("Umid. terr. s.2 = ");
  Serial.print(umid_terreno2);
  Serial.println(" %");

  Serial.print("Umid. terr. s.3 = ");
  Serial.print(umid_terreno3);
  Serial.println(" %");

  Serial.print("Illuminazione = ");
  Serial.print(luminosita);
  Serial.println(" lux");

  Serial.print("Anid. carbonica = ");
  Serial.print(anid_carbonica);
  Serial.println(" ppm");
  
  Serial.print("Particolato = ");
  Serial.print(particolato);
  Serial.println(" ppm");

  Serial.print("Stato AQ = ");
  Serial.println(aq_stato);
  Serial.print("Lettura AQ = ");
  Serial.println(aq_valore);
}
