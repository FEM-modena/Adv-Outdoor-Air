/* 
  Green Air Explorer Kit
  Future Education Modena 2022

  Monitora: 
  - Umidità/Temperatura (Shield MKR Env)
  - Illuminazione (Shield MKR Env)
  - Umidità  del terreno (Grove Sensore Capacitivo analogico)
  - Anidride Carbonica (Grove SCD-30)
  - Qualità dell'Aria - inquinanti (Grove Air Quality v1.3)
  - Particolato (Grove Dust Sensor)
*/

///// NON SPOSTARE QUESTE DEFINIZIONI  /////
//*******************************************
#define SECRET_SSID "FEM_WiFi"
#define SECRET_PASS "0h4orXc@yS3do"
#define CHIAVE_CLOUD "FEMGreenAirExplorer_serra"

char dboard_server[] = "demo.thingsboard.io"; // Indirizzo IP/Internet del Dashboard Server
int dboard_port = 80;                         // Porta TCP del server

//Variabili
float temp_aria = 0;
float umid_aria = 0;
float luminosita = 0;
int umid_terreno1 = 0;
int umid_terreno2 = 0;
int umid_terreno3 = 0;
float anid_carbonica = 0;
float particolato = 0;
int aq_valore;
int aq_tend;

//Collegare una resistenza da 1K o il Grove Led Rosso
#define PIN_LED1 5
//Collegare RESET a questo pin con un jumper
#define PIN_RESET 4

//Collegamento alla funzione del led
void accendi_LED_per(byte volte);

//Collegamento alla piattaforma GL-Blocks
#include "GL-Blocks-WiFi.h"
#include "GL-Blocks-Dashboard-GAX.h"
//*******************************************
///// NON SPOSTARE QUESTE DEFINIZIONI  /////

#include <Wire.h>  

//Libreria ENV Shield di Arduino
//Installare da "Gestione Librerie"
#include <Arduino_MKRENV.h>
//https://github.com/Seeed-Studio/Seeed_SCD30
//Libreria per Grove CO2 Hum/Temp SCD30
//ATTENZIONE: NON INSTALLARE DA GESTIONE LIBRERIE: scaricare da repo Github
//e copiarlo nella directory 'libraries' di Arduino!

#include "SCD30.h"
float last_ppmCO2;

//Libreria per Grove Air Quality Sensor 1.3
//ATTENZIONE: NON INSTALLARE DA GESTIONE LIBRERIE: scaricare da repo Github
//e copiarlo nella directory 'libraries' di Arduino!
//https://github.com/Seeed-Studio/Grove_Air_quality_Sensor
#include "Air_Quality_Sensor.h"
//Collegare il sensore Air Quality Sensor al connettore analogico indicato
#define PIN_AIR_QUALITY A3
AirQualitySensor sensore_aq = AirQualitySensor(PIN_AIR_QUALITY);
String aq_stato;

//Collegare il sensore capacitivo ai connettori analogici indicato
#define PIN_UMIDITA_1 A1
#define PIN_UMIDITA_2 A4
#define PIN_UMIDITA_3 A5

//Collegare il sensore di particolato al connettore digitale indicato
#define PIN_SENSORE_PARTICOLATO 3
//Periodo di lettura
int INTERVALLO_PARTICOLATO = 60000; //In millisecondi: 1000 = 1 sec

//Misura del particolato proporzionale alla durata dell'impulso
unsigned long durataImpulsoLow = 0;
//Tempo di ultima lettura particolato
long ultimaLetturaDust;

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

  //Avvio del Grove CO2 Temp/Hum SCD30
  scd30.initialize();

  //Avvio del Grove Air Quality Sensor
  sensore_aq.init();

  //Inizio temporizzazione particolato
  pinMode(PIN_SENSORE_PARTICOLATO, INPUT);
  ultimaLetturaDust = millis();

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
