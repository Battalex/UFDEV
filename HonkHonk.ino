/**\file Evaluation_B1A.ino 
 * 
 * Consignes :
 * - Nom du fichier : Nom_Prénom_B1A.ino
 * - Date limite de remise : 12h00
 * - Nombre de fichier : un seul fichier .ino
 * 
 * Pour cette évaluation nous allons réaliser une plante connectée avec les fonctionnalités suivantes :
 * - Wifi : 2 Points
 *   Ma plante connectée se connecte au réseau WiFi sur le SSID d'YNOV "YNOVAIX IOT" avec le mot de 
 *   passe "9xUNb8mk7Vs3A3Y" (ou celui de votre connexion partagée).
 * - Capteurs : 4 points
 *   Ma plante connectée utilise un capteur de température et d'humidité de l'air connecté sur la broche
 *   D0 ainsi qu'un capteur d'humidité du sol connecté sur la broche D2. Ma plante relève les données 
 *   et les stocke dans des variables globales (présentes dans le code de départ):
 *   - fHumidity pour l'humidité de l'air, en pourcentage (entre 0 et 100%),
 *   - fTemperature pour la température de l'air, en degrés Celcius (entre -20 et 50°C),
 *   - iSoilMoisture pour l'humidité du sol, en pourcentage (entre 0 et 100%).
 * - Actuateur : 2 points
 *   Ma plante connectée dispose d'un actuateur qui est une pompe à eau; elle sera représentée par
 *   un LED dans le cadre de cette évaluation. On utilisera la LED du NodeMCU connectée sur la Pin D4.
 *   Il faut ici définir les informations et initialiser l'actuateur (la pompe) à l'état OFF au démarrage
 *   de la plante.
 * - Web Server : 3 points
 *   Ma plante connectée dispose d'un serveur web, accessible depuis son adresse IP sur le port 80 et
 *   affiche :
 *   - Le SSID sur lequel elle est connectée,
 *   - Les dernières valeurs des 3 données de capteur,
 *   - Un bouton qui permet d'activer la pompe à eau (la LED dans notre cas). Quand on allume la pompe 
 *     la LED s'éteind et quand j'arrête la pompe la LED s'allume,
 *   - La page web se rafraichit toutes les 10 secondes.
 * - Adafruit IO : 3 points
 *   Ma plante est connectée à la plateforme Adafruit IO. Un tableau de bord permet de visualiser : 
 *   - Les courbes d'évolution des 3 données,
 *   - Les valeurs instantannées des 3 données sont visibles par des jauges,
 *   - Un bouton permet de visualiser l'état de l'actuateur (pompe ou LED) et d'en changer l'état,
 *   - Un slider permet de configurer un niveau de température entre 0 et 50 degrés,
 *   - Un slider permet de configurer un niveau d'humidité de l'air entre 0 et 100 %.
 * - Règle métier : 3 points
 *   Ma plante connectée vérifie toutes les 30 secondes si 
 *   - la température de l'air n'est pas au dessus du niveau défini par le slider sur le tableau de bord
 *   - l'humidité du sol n'est pas au dessous du niveau défini par le slider sur le tableau de bord
 *   - Si un de ces critères n'est pas respecté alors elle active la pompe (donc éteint la LED)
 *   - Si les 2 critères sont respectés alors la pompe est arrêtée
 * - Configuration : 3 points
 *   Ma plante connectée dispose d'un fichier de configuration (sur le SPIFFS) contenant :
 *   - Le type de plante
 *   - Le taux optimal d'humidité du sol
 *   - La température athmosphérique optimale
 * - Réinitialisation : 2 points
 *   Sur la page principale du serveur web de ma plante, un bouton permet de réinitialiser les valeurs
 *   des seuils définis par les sliders 
 * - Commentaires :
 *   0,5 point pour chaque fonctionnalité implémentée correctement documentée (permettant de comprendre 
 *   les étapes et facilitant la lecture du code) et les logs sont utilies et explicites
 *   
*/

/**
 * Liste des fonctionnalités que j'ai implémentées :
 * Attention à retirer les fonctionnalités que vous n'avez pas implémentées : -1 point par fonctionnalité
 * affichée comme réalisée et qui n'est pas opérationnelle.
 * 
 * - Wifi :             2 (+0,5) points
 * - Capteurs :         4 (+0,5) points
 * - Actuateur :        2 (+0,5) points
 * - Web Server :       3 (+0,5) points
 * - Adafruit IO :      3 (+0,5) points
 * - Règle métier :     3 (+0,5) points
 * - Configuration :    3 (+0,5) points
 * - Réinitialisation : 2 (+0,5) points
 * - Consignes :        1 point
 */
#include <ESP8266WiFi.h>             //https://github.com/esp8266/Arduino
#include "DHT.h"                       // https://github.com/adafruit/DHT-sensor-library
#include <DHT_U.h>                     // https://github.com/adafruit/Adafruit_Sensor

/* DEFINITION DES VARIABLES GLOBALES */
#define DHT_U 1                        // 0 si DHT "classique" / 1 pour DHT Unified Sensor
#define DHT_PIN    D2                  // PIN numérique du DHT
#define DHT_TYPE   DHT11               // Type de capteur DHT : DHT11 ou DHT22
#define SOIL_DIGITAL_PIN  D0           // PIN numérique de l'information de sécheresse
#define SOIL_ANALOG_PIN   A0           // PIN digitale du niveau d'humidité du sol
#define MOISTURE_HIGH     1024         // Calibration du capteur : valeur haute (sec)
#define MOISTURE_LOW      0            // Calibration du capteur : valeur basse (dans l'eau)
#define pompePin D4                    // D4 correspond également à la LED de l'ESP8266 (HIGH & LOW inversés)
uint32_t delayMS = 5000;               // Délai entre 2 mesures

/* DEFINITION DES ALIAS DE DEBUG */
#define MYDEBUG 1
#ifdef MYDEBUG
 #define MYDEBUG_PRINT(x)     Serial.print (x)
 #define MYDEBUG_PRINTLN(x)   Serial.println (x)
#else
 #define MYDEBUG_PRINT(x)
 #define MYDEBUG_PRINTLN(x) 
#endif

/* INSTANCIATION DES CAPTEURS DHT */
#if DHT_U
  DHT_Unified myDht(DHT_PIN, DHT_TYPE); // Instanciation du DHT Unified
#else
  DHT myDht(DHT_PIN, DHT_TYPE);         // Instanciation du DHT "classique"
#endif

float fHumidity;                        // Taux d'humidité de l'air
float fTemperature;                     // Température de l'air
int iSoilMoisture;                      // Taux d'humidité du sol
int iDigitalThreshold;                  // Seuil d'humidité du sol
char* ssid     = "Livebox-8B36";         // Le SSID du wifi auquel on veut se connecter
char* password = "YvPxGC4Q13";     // Son mot de passe
WiFiServer server(80);                  // Serveur web sur le port 80
String header;                          // Requête HTTP 
String strPompeState = "off";           // Etat de l'actuateur

// CONNEXION AU WIFI
void connexionWiFi(char* ssid, char* password){
  MYDEBUG_PRINTLN(); // Saut de ligne
  MYDEBUG_PRINT("-WIFI : Connexion en cours à ");
  MYDEBUG_PRINT(ssid);

  WiFi.mode(WIFI_STA); // On fixe le mode client à la carte pour éviter qu'elle se comporte comme un point d'accès et qu'il y ait des conflits
  WiFi.begin(ssid, password); // On se connecte à l'aide des identifiants stockés dans les variables

  while (WiFi.status() != WL_CONNECTED) { // Tant que l'on n'est pas connecté
    delay(500); // On attend 0.5 secondes
    MYDEBUG_PRINT("."); // On affiche un point
  }
  MYDEBUG_PRINTLN(""); // Saut de ligne
  MYDEBUG_PRINT("-WIFI : connecté avec l'adresse IP : ");
  MYDEBUG_PRINTLN(WiFi.localIP()); // On affiche l'adresse IP qui nous a été accordée
}

void setupDHT(){
  myDht.begin(); // On démarre la communication avec le capteur DHT

#if DHT_U
  MYDEBUG_PRINTLN("DHT Unified Sensor");
  // Informations sur le capteur de température
  sensor_t sensor;
  myDht.temperature().getSensor(&sensor);
  MYDEBUG_PRINTLN("------------------------------------");
  MYDEBUG_PRINTLN("Temperature");
  MYDEBUG_PRINT  ("Sensor:       "); MYDEBUG_PRINTLN(sensor.name);
  MYDEBUG_PRINT  ("Driver Ver:   "); MYDEBUG_PRINTLN(sensor.version);
  MYDEBUG_PRINT  ("Unique ID:    "); MYDEBUG_PRINTLN(sensor.sensor_id);
  MYDEBUG_PRINT  ("Max Value:    "); MYDEBUG_PRINT(sensor.max_value); MYDEBUG_PRINTLN(" *C");
  MYDEBUG_PRINT  ("Min Value:    "); MYDEBUG_PRINT(sensor.min_value); MYDEBUG_PRINTLN(" *C");
  MYDEBUG_PRINT  ("Resolution:   "); MYDEBUG_PRINT(sensor.resolution); MYDEBUG_PRINTLN(" *C");  
  MYDEBUG_PRINTLN("------------------------------------");
  // Informations sur le capteur d'humidité
  myDht.humidity().getSensor(&sensor);
  MYDEBUG_PRINTLN("------------------------------------");
  MYDEBUG_PRINTLN("Humidity");
  MYDEBUG_PRINT  ("Sensor:       "); MYDEBUG_PRINTLN(sensor.name);
  MYDEBUG_PRINT  ("Driver Ver:   "); MYDEBUG_PRINTLN(sensor.version);
  MYDEBUG_PRINT  ("Unique ID:    "); MYDEBUG_PRINTLN(sensor.sensor_id);
  MYDEBUG_PRINT  ("Max Value:    "); MYDEBUG_PRINT(sensor.max_value); MYDEBUG_PRINTLN("%");
  MYDEBUG_PRINT  ("Min Value:    "); MYDEBUG_PRINT(sensor.min_value); MYDEBUG_PRINTLN("%");
  MYDEBUG_PRINT  ("Resolution:   "); MYDEBUG_PRINT(sensor.resolution); MYDEBUG_PRINTLN("%");  
  MYDEBUG_PRINTLN("------------------------------------");
  delayMS = sensor.min_delay/1000; // On calcule le délai entre 2 mesures par rapport au délai minimum du capteur
  MYDEBUG_PRINT  ("Delay:   "); MYDEBUG_PRINT(delayMS); MYDEBUG_PRINTLN(" ms");   // On affiche le délai en ms
  MYDEBUG_PRINTLN("------------------------------------");
#endif
}

String getDhtData(){
  // Délai minimum entre 2 mesures
  MYDEBUG_PRINT("-DHT : Délai entre 2 mesure [");
  MYDEBUG_PRINT(delayMS);
  MYDEBUG_PRINTLN("] ms");
  delay(delayMS);

#if DHT_U
  sensors_event_t event; // On récupère l'évènement de capture
  myDht.temperature().getEvent(&event); // On récupère les informations de l'évènement température
  if(isnan(event.temperature)){ // Si la valeur temperature de l'évènement n'est pas un nombre
    MYDEBUG_PRINTLN("-DHT : Erreur de lecture de la température du capteur DHT !"); // Erreur
  }else{ // Si pas d'erreur
    MYDEBUG_PRINT("-DHT : [");
    MYDEBUG_PRINT(event.temperature); // On l'affiche
    MYDEBUG_PRINTLN("°C] température !");
  }
  String values = String(event.temperature) + "°C";
  myDht.humidity().getEvent(&event); // On récupère les informations de l'évènement humidité
  if (isnan(event.relative_humidity)){ // Si la valeur relative_humidity de l'évènement n'est pas un nombre
    MYDEBUG_PRINTLN("-DHT : Erreur de lecture de l'humidité du capteur DHT !"); // Erreur
  }else{ // Si pas d'erreur
    MYDEBUG_PRINT("-DHT : [");
    MYDEBUG_PRINT(event.relative_humidity); // On l'affiche
    MYDEBUG_PRINTLN("%] Humidité !");
  }
  values += " | " + String(event.relative_humidity) + "%";
  return values;
#else
  // Lecture des données sur le capteur
  fHumidity = myDht.readHumidity(); // On stocke l'humidité dans la variable
  fTemperature = myDht.readTemperature(); // On stocke la température dans la variable

  // Vérification que la lecture est correcte
  if (isnan(fHumidity) || isnan(fTemperature)){ // Si l'une des valeurs ou les valeurs retournées sont fausses
    MYDEBUG_PRINTLN("-DHT : Erreur de lecture du capteur DHT !");
    return; // On quitte la fonction
  }
  // Affichage
  MYDEBUG_PRINT("-DHT : [");
  MYDEBUG_PRINT(fHumidity);
  MYDEBUG_PRINTLN("%] humidité !");
  MYDEBUG_PRINT("-DHT : [");
  MYDEBUG_PRINT(fTemperature);
  MYDEBUG_PRINTLN("°C] température !");
#endif
}

void setupSoilSensor(){
  pinMode(SOIL_ANALOG_PIN, INPUT); // On initialise le pin analogique du capteur d'humidité en tant qu'entrée
  pinMode(SOIL_DIGITAL_PIN, INPUT); // On initialise le pin numérique du capteur d'humidité en tant qu'entrée
}

int getSoilData(){
  // HUMIDITE - ANALOG
  iSoilMoisture = analogRead(SOIL_ANALOG_PIN); // On récupère l'information renvoyée par le capteur analogique
  iSoilMoisture = map(iSoilMoisture,MOISTURE_HIGH,MOISTURE_LOW,0,100); // On transforme les valeurs reçues en pourcentage
  MYDEBUG_PRINT("-SOL : [");
  MYDEBUG_PRINT(iSoilMoisture); // On l'affiche
  MYDEBUG_PRINTLN("%] humidité !");  

  // SEUIL - DIGITAL
  iDigitalThreshold = digitalRead(SOIL_DIGITAL_PIN); // On récupère l'information renvoyée par le capteur numérique
  if(iDigitalThreshold == HIGH){ // Si le capteur renvoie HIGH
    MYDEBUG_PRINTLN("-SOL : Le sol est sec !");
  } else { // Sinon
    MYDEBUG_PRINTLN("-SOL : Le sol n'est pas trop sec");
  }
  return iSoilMoisture; // On renvoie la variable pour pouvoir l'afficher sur le site
}

void setupPompe(){
  pinMode(pompePin, OUTPUT);   // Configuration de la LedPin en sortie
  digitalWrite(pompePin, HIGH); // Initialisation de l'état de la LED à LOW (inversés)
}

void setupWebServer(){
  // On a besoin d'une connexion WiFi !
  if (WiFi.status() != WL_CONNECTED){connexionWiFi(ssid, password);}  // Connexion WiFi
  MYDEBUG_PRINTLN("-Web Server : Démarrage");
  server.begin();                                   // Démarrage du serveur
}

void loopWebServer(){
  WiFiClient client = server.available();           // Attente de clients
  if (client) {                                     // Si un nouveau client se connecte
    MYDEBUG_PRINTLN("-Web Server : Nouveau client");
    String currentLine = "";                        // Initialisation d'un String pour les données du client
    while (client.connected()) {                    // Boucle tant que le client est connecté
      if (client.available()) {                     // S'il y a des données du client
        char c = client.read();                     // Lecture d'un octet
        header += c;                                // Stockage dans le header
        if (c == '\n') {                            // Si l'octet est un retour à la ligne
          // Si la ligne courante est vide, on a deux caractères de fin de ligne
          // on identifie donc la fin de la requête HTTP, il faut donc désormais répondre
          if (currentLine.length() == 0) {
            // Construction de la réponse avec le HTTP header qui commence par la code retour
            // (e.g. HTTP/1.1 200 OK) et une description du contenu pour que le client sache ce
            // qui lui arrive, et finalement une ligne vide pour indiquer la fin
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // Analyse de la requête pour savoir si une action a été demandée
            if(header.indexOf("GET /LED/on") >= 0){
              MYDEBUG_PRINTLN("-Web Server : Actuateur ON");
              strPompeState = "on";
              digitalWrite(pompePin, LOW);
            }else if(header.indexOf("GET /LED/off") >= 0){
              MYDEBUG_PRINTLN("-Web Server : Actuateur OFF");
              strPompeState = "off";
              digitalWrite(pompePin, HIGH);
            }

            String soilValues = String(getSoilData()); // On récupère la valeur de l'humidité du sol du capteur
            String DHTValues = getDhtData(); // On récupère les valeurs de l'humidité et de la température de l'air du capteur
            
            if(header.indexOf("GET /VALUES/reset")){
              MYDEBUG_PRINTLN("-Web Server : Réinitialisation des valeurs");
              soilValues = String(getSoilData()); // On récupère la valeur de l'humidité du sol du capteur
              DHTValues = getDhtData(); // On récupère les valeurs de l'humidité et de la température de l'air du capteur
            }

            // Affichage de la page web
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<meta HTTP-EQUIV=\"Refresh\" content=\"10\">");
            client.println("<meta charset=\"utf-8\"");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");
            client.println("<body><h1>My Web Server</h1>");
            client.println("<h1> HONK HONK </h1>");
            client.println("<p><b>SSID:</b> " + String(ssid) + "</p>");
            client.println("<p><b>Capteur d'humidité du sol:</b> " + soilValues + " %</p>");
            client.println("<p><b>Capteur de température et d'humidité de l'air:</b> " + DHTValues + "</p>");
            client.println("<p><b>Etat de l'actuateur :</b> " + strPompeState + "</p>");
            // Affichage du bouton selon l'état de l'actuateur
            if (strPompeState=="off") {
              client.println("<p><a href=\"/LED/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/LED/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
            client.println("<p><a href=\"/VALUES/reset\"><button class=\"button button2\">RESET</button></a></p>");
            client.println("</body></html>");
            // On termine avec une ligne vide pour indiquer la fin de la réponse            
            client.println();
            break;
          } else { // C'est une fin de ligne, on "nettoie" la ligne courante
            currentLine = "";
          }
        } else if (c != '\r') {  // C'est autre chose qu'un retour à la ligne
          currentLine += c;      // Alors on l'ajoute à la ligne courante
        }
      }
    }
    // "Nettoyage" du header
    header = "";
    // Fermeture de la connection client
    client.stop();
    MYDEBUG_PRINTLN("-Web Server : Client déconnecté");
  }
}

/**
 * Fonction setup qui est appelée au démarrage de la carte, j'initialise ici les variables et
 * les capteurs. Cette fonction n'est appelée qu'une seule fois au démarrage (ou Reset).
 */
void setup() {
  Serial.begin(115200);             // Démarrage du port série avec l'ordinateur à la vitesse de 115200 bauds
  MYDEBUG_PRINTLN("--SETUP--");
  connexionWiFi(ssid, password);    // On se connecte au réseau wifi
  setupWebServer();
  setupDHT();                       // On initialise la connexion au capteur DHT
  setupSoilSensor();                // On initialise la connexion au capteur d'humidité de sol
  setupPompe();                     // On initialise la liaison à la "pompe à eau"
}

/**
 * Fonction loop qui est appellée en boucle dès que la fonction setup est terminée
*/
void loop() {
  loopWebServer();
  getSoilData(); // On récupère l'humidité du sol
  getDhtData(); // On récupère la température et l'humidité de l'air
}
