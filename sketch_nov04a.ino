#include <SimpleTimer.h>
#define PI 3.1415926535897932384626433832795


float vitesse = 2; // consigne de vitesse
int r_reduc = 20; // rapport de réduction
const float cad_ech = 0.01; // cadence d'échantillonage
SimpleTimer timer; // timer

float erreur_precedente=0;
float somme_erreur = 0;
volatile int compteur=0;
  
void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT);// pin de consigne de PWM (rotation)
  pinMode(6, OUTPUT);// pin de consigne de PWM (rotation)
  pinMode(3,OUTPUT); // pin du PWM (rapport cyclique)
  
  pinMode(2,INPUT); // pin d'interruption
 
  digitalWrite(5,HIGH);

  // on incrémente dès qu'il y a un signal
  attachInterrupt(digitalPinToInterrupt(2), comptage, CHANGE);

  // on asservit tous les 0.01 s
  timer.setInterval(cad_ech*1000, asservissement);
}


void loop() {
  timer.run();
}

// incrémentation dès qu'il y a un signal
void comptage(){
 compteur++;
}

void asservissement(){
  
  int nb_impulsions=compteur;
  
  // calcul de la vitesse du moteur
  float vitesse_moteur = nb_impulsions/(r_reduc*cad_ech*11);
  compteur=0;
  
  float erreur = vitesse-vitesse_moteur; // calcul erreur 
  float somme_erreur=somme_erreur+erreur;
  float consigne = 60*erreur + 5*(erreur-erreur_precedente)+ 1.5*somme_erreur; // kp = 60 ; kd=5 ; ki = 1.5
  float erreur_precedente = erreur;
  
  int commande = (int)(255*consigne/5);
   if (commande>255) {
        commande = 255;
    } 
   if (commande<0) {
        commande = 0;
    } 

  // Commande du PWM
  analogWrite(3, commande);
}
