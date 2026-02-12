#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>  


// ---Declaration des Broches---
unsigned char PWMGplus = 17;                                    // PWM (vitesse) du moteur gauche
unsigned char PWMDplus = 16;                                    // PWM (vitesse) du moteur droit

unsigned char PWMGmoins = 4;                                    // PWM (vitesse) du moteur gauche
unsigned char PWMDmoins = 19;                                    // PWM (vitesse) du moteur droit

char Batterie = 25;                                         // Broche de lecture de la batterie 

// ---Declaration des Objets---
BluetoothSerial SerialBT;                                   // Objet Bluetooth Serial
Adafruit_MPU6050 mpu;                                       // Objet pour le capteur MPU6050

// ---Declaration des Variables---
float TetaG,TetaW,angle;
float TetaWF,TetaGF;                     
char FlagCalcul = 0;
float Ve, Vs = 0;
float Te = 10;                                              // période d'échantillonage en ms
float Tau = 1000;                                           // constante de temps du filtre en ms
float A, B;                                                 // coefficient du filtre

float anglePositifMax = 0.0;
float angleNegatifMax = 0.0;                     
char kp = 2;                                                // coefficient de proportionnalité du correcteur
unsigned short valPWM=0, maxPWM = 1024;

float R1= 22000.0;                                          // résistance de 22 kohms
float R2= 10000.0;                                          // résistance de 10 kohms
float valeurbatterie;

// ---Definition des PWM---
unsigned int frequence = 20000;                     // Fréquence de 20 kHz
unsigned char canal0 = 0;                                   // Canal 0 pour le moteur gauche
unsigned char canal1 = 1;                                   // Canal 1 pour le moteur droit
unsigned char canal2 = 2;                                   // Canal 2 pour le moteur gauche
unsigned char canal3 = 3;                                   // Canal 1 pour le moteur droit
unsigned char resolution = 10;                              // Résolution de 10 bits (valeurs de 0 à 1023)


void controle(void *parameters)
{
  TickType_t xLastWakeTime;                                 // Variable pour stocker le temps de réveil de la tâche
  xLastWakeTime = xTaskGetTickCount();                      // Initialisation du temps de réveil de la tâche
  while (1)
  {
    sensors_event_t a, g, temp;                             // Création d'objets pour stocker les données du capteur                        
    mpu.getEvent(&a, &g, &temp);                            // Lecture des données du capteur
   
    TetaG=-atan2(a.acceleration.y, a.acceleration.x);       // Angle du gyroscope en radian
    TetaGF = A * TetaG + B * TetaGF;                        // Angle du gyroscope filtré en radian
   
    TetaW  =  g.gyro.z * Tau/1000;                          // angle de l'accélération en radian  
    TetaWF = A* TetaW + B*TetaWF;                           // angle de l'accélération filtré en radian
 
    angle = (TetaWF + TetaGF) * 180 / PI;                   // angle de l'inclinaison du gyropode en degrés    

    if(angle > 0 && angle > anglePositifMax) anglePositifMax = angle;     // Calcule Angle maximum positif
    if(angle < 0 && angle < angleNegatifMax) angleNegatifMax = angle;     // Calcule Angle maximum negatif

    if (angle < 4.0 && angle > -4.0){                        // Arret
      ledcWrite(canal0, 0);
      ledcWrite(canal1, 0);      
      ledcWrite(canal2, 0);
      ledcWrite(canal3, 0);
    }

    if (angle > 0 && angle < anglePositifMax){              // Avant
      valPWM = kp*(angle/anglePositifMax)*maxPWM;
      if(valPWM >maxPWM) valPWM = maxPWM;                             // Saturation PWM Max
      if(valPWM < 0) valPWM = 0;                                      // Saturation PWM Min
      ledcWrite(canal0, valPWM);
      ledcWrite(canal1, valPWM);      
      ledcWrite(canal2, 0);
      ledcWrite(canal3, 0);
    }

    if (angle < 0 && angle > angleNegatifMax){              // Arrière
      valPWM = kp*(angle/angleNegatifMax)*maxPWM;           // Calcul de la valeur PWM en fonction de l'angle d'inclinaison
      if(valPWM >maxPWM) valPWM = maxPWM;                             // Saturation PWM Max
      if(valPWM < 0) valPWM = 0;                                      // Saturation PWM Min
      ledcWrite(canal0, 0);
      ledcWrite(canal1, 0);
      ledcWrite(canal2, valPWM);
      ledcWrite(canal3, valPWM);
    }
    
    FlagCalcul = 1;                                         // Indicateur que les calculs sont terminés
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));     // Attente jusqu'au prochain cycle d'exécution de la tâche
  }
}

/*void Vin(void *parameters)                                  // Tâche pour la lecture de la tension de la batterie
{
  Ve = 1;
  while (1)
  {
    valeurbatterie=(((3.3/4095.0)*analogRead(Batterie)*(R1+R2))/R2)+0.3;     // Calcul de la valeur de la batterie en volts

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
*/
void setup()
{
  Serial.begin(115200);
  Serial.printf("Bonjour \n\r");
  
  SerialBT.begin("ESP32test");                              // Bluetooth device name
  SerialBT.println("Hello from ESP32!");                    // Message de bienvenue pour le Bluetooth

  // Configuration de la PWM
  ledcSetup(canal0, frequence, resolution);               
  ledcSetup(canal1, frequence, resolution);         
 
  ledcSetup(canal2, frequence, resolution);               
  ledcSetup(canal3, frequence, resolution);      
  
 
  // Liaison des canaux PWM aux broches
  ledcAttachPin(PWMGplus, canal0);        
  ledcAttachPin(PWMDplus, canal1);
  ledcAttachPin(PWMGmoins, canal2);
  ledcAttachPin(PWMDmoins, canal3);
  

  // Initialisation du capteur MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      10,         // tres haut niveau de priorite
      NULL        // descripteur
  );
  /*xTaskCreate(
      Vin,        // nom de la fonction
      "Vin",      // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,      // parametre
      1,          // bas niveau de priorite
      NULL        // descripteur
  );*/

  // calcul coeff filtre
  Serial.begin(115200);
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;
}

void reception(char ch)
{
  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }

    if (commande == "Te")
    {
      Te = valeur.toInt();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    
    if (commande == "kp")
    {
      kp = valeur.toInt();
    }

    if (commande == "maxPWM")
    {
      maxPWM = valeur.toInt();
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void loop()
{
  if (FlagCalcul == 1)
  {
    Serial.printf("Angle : %.2f | anglePositif: %.2f | angleNegatif: %.2f | valPWM : %d\n",angle, anglePositifMax, angleNegatifMax, valPWM); // Affichage des angles sur le moniteur série
    //Serial.printf("valBatterie: %.4f \n", valeurbatterie); // Affichage de la valeur de la batterie sur le moniteur série
    FlagCalcul = 0;
  }
  
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}