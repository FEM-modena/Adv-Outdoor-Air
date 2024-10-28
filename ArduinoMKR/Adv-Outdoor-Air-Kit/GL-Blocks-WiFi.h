/****
 * GL Blocks
 * Piattaforma per la programmazione a blocchi
 * Green Lab - Future Education Modena 2020-2021
 * 
 * Supporto WiFi
 */

#include <WiFiNINA.h>
WiFiClient client;
char ssid[] = SECRET_SSID; // SSID (name)
char pass[] = SECRET_PASS; // WIFI pwd

#define MAX_TENTATIVI_WIFI 10

/*************************************
 Funzioni dei BLOCCHI WiFi
 *************************************/

/**
 * Stampa lo stato della WiFi
 */
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/**
 * Legge e riporta lo stato della connessione WiFi
 */
 int report_wifi_status(void) 
 {
   /*
        WIFI STATUS:
        
        WL_NO_SHIELD = 255,
        WL_NO_MODULE = WL_NO_SHIELD,
        WL_IDLE_STATUS = 0,
        WL_NO_SSID_AVAIL = 1,
        WL_SCAN_COMPLETED = 2,
        WL_CONNECTED = 3,
        WL_CONNECT_FAILED = 4,
        WL_CONNECTION_LOST = 5,
        WL_DISCONNECTED = 6,
        WL_AP_LISTENING = 7,
        WL_AP_CONNECTED = 8,
        WL_AP_FAILED = 9
   */
  int wifi_status = WiFi.status();
  Serial.print("Stato WiFi:");
  switch (wifi_status)
  {
  case 1:
    Serial.println("SSID della rete non valido");
  break;
  case 2:
    Serial.println("Scan compeltato");
  break;
  case 3:
    Serial.println("Connessa");
  break;
  case 4:
    Serial.println("Connessione fallita");
  break;
  case 5:
    Serial.println("Connessione persa");
  break;
  case 6:
    Serial.println("Disconessa");
  break;
  }
  return wifi_status;

 }


/**
 * Si connette alla WiFi configurata
 */
void Connetti_WIFI(void)
{  
  int wifi_status = report_wifi_status();
  // if connected, exit
  if (wifi_status == WL_CONNECTED) return;

  // Se il WiFi Module non risponde, reset di Arduino
  if (wifi_status == WL_NO_MODULE) {
    Serial.println("Comunicazione con il modulo WiFi fallita.");
    digitalWrite(PIN_LED1, HIGH);
    delay(3000); //LED Fisso per 3 secondi
    //Resetta la scheda se PIN_RESET è cablato al RESET di MKR
    digitalWrite(PIN_RESET, LOW);
  }
  
  // Tentativo di connessione al WiFi
  int tentativo = 0;
  while (wifi_status != WL_CONNECTED) {    
    accendi_LED_per(1);
    Serial.print("Provo a connettermi alla WiFi SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    wifi_status = WiFi.begin(ssid, pass);
    report_wifi_status();    
    // wait 5 seconds for connection:
    delay(4000);
    // max 5 tentativi, poi reset di Arduino
    if (++tentativo == MAX_TENTATIVI_WIFI) {
      Serial.println("Massimo numero di tentativi raggiunto.");
      digitalWrite(PIN_LED1, HIGH);
      delay(3000); //LED Fisso per 3 secondi
      //Resetta la scheda se PIN_RESET è cablato al RESET di MKR
      digitalWrite(PIN_RESET, LOW);
    }
  }

  Serial.println("Connesso alla WiFi!");
  delay(2000);
  printWifiStatus();
  
}
