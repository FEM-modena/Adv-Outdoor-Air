/****
 * GL Blocks 
 * Advanced Outdoor Air
 * Green Lab - Future Education Modena 2024-2025
 */
 
unsigned long lastConnectionTime = 0;   // tempo dell'ultima connessione al server

//Libreria JSON
#include <ArduinoJson.h>  
//vedi https://arduinojson.org/v5/assistant/
const size_t json_capacity = JSON_OBJECT_SIZE(10) + 133; 
StaticJsonDocument<json_capacity> doc;

/*************************************
 Funzioni dei BLOCCHI
 *************************************/
/**
 * Invia i dati con interfaccia HTTP
 */
void Trasmetti_Dati_Cloud() 
{

/**
 * https://thingsboard.io/docs/reference/http-api/
 * 
 * POST http://localhost:8080/api/v1/$ACCESS_TOKEN/telemetry --header "Content-Type:application/json"
 * 
 * JSON:
{
  "pm10": <mg/m3-pm10>,
  "pm2_5": <mg/m3-pm2_5>,
  "ozono": <ppm ozono>,
  "oss_carb": <ppm_ossido_carb>,
  "ammon": <ppm ammoniaca>,
  "bioss_az": <ppm_bioss_azoto>,
  "metano": <ppm_metano>
}
 */

  // Carica le misure nel documento JSON
  doc["pm10"] = pm10;
  doc["pm2_5"] = pm2_5;
  doc["ozono"] = ozono;
  doc["oss_carb"] = oss_carbonio;
  doc["ammon"] = ammoniaca;
  doc["bioss_az"] = bioss_azoto;
  doc["metano"] = metano;  


  // Close any connection before send a new request.
  // This will free the socket on the Nina module
  client.stop();

  Serial.println("\nConnessione al server IoT");
  if (client.connect(dboard_server, dboard_port)) 
  {
    Serial.println("Connesso al Cloud IoT");
    // Make a HTTP request:
    client.println("POST /api/v1/" CHIAVE_CLOUD "/telemetry HTTP/1.1"); 
    String host_string = String("Host: ") + String(dboard_server);
    client.println(host_string);  
    client.println("Connection: close");  
    client.print("Content-Length: ");  
    client.println(measureJson(doc));  
    client.println("Content-Type: application/json");  
    // Terminate headers with a blank line
    client.println();
    // Send JSON document in body
    serializeJson(doc, client);

    // note the time that the connection was made:
    lastConnectionTime = millis();
    
    Serial.println("Misure inviate al Cloud IoT");
    accendi_LED_per(3);
    
  } else {
    
    // if you couldn't make a connection:    
    Serial.println("Connessione al Cloud IoT non riuscita.");
    accendi_LED_per(4);
  }
  
}
