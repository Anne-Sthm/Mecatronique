#include <Arduino.h>

#define consigne 2 // consigne de vitesse
#define r_reduc 20 // rapport de réduction
#define impulsion 11 // nb impulsion par tour 
#define cad_ech 0.01 // cadence d'échantillonage

// PINS Moteur 1
#define M1_pwm 10
#define M1_sens_a 4
#define M1_sens_b 5
#define M1_interruption 3

// PINS Moteur 2
#define M2_pwm 6
#define M2_sens_a 1
#define M2_sens_b 7
#define M2_interruption 2

// PINS Capteur 1
#define trig_a 8
#define echo_a 11

//PINS Capteur 2
#define trig_b 12
#define echo_b 13

const float vson = 340.0/1000; // vitesse du son

//PID
float erreur;
float vitesse;
float commande;
float erreur_precedente = 2;
float somme_erreur = 0;
float current_speed;

//Obstacles
long temps;
float distance_A;
float distance_B;
double rep_capteur;
volatile int tick_M1 = 0; // compte le nombre d'impulsions
volatile int tick_M2 = 0; // compte le nombre d'impulsions


// incrémentation dès qu'il y a un signal
void comptage_M1(){
 tick_M1++;
}

void comptage_M2(){
 tick_M2++;
}

ISR(TIMER1_COMPA_vect){

  cli();

  /************ MOTEUR 1*******************/

  // Calcul de la vitesse du moteur
  current_speed = tick_M1/(r_reduc*cad_ech*impulsion);

  // Calcul de l'erreur
  erreur = rep_capteur-current_speed; 
  somme_erreur+=erreur;

  // Calcul de la commande
  commande = 0.05714285714*erreur 
  + 0.45454545454*somme_erreur 
  + 0*(erreur-erreur_precedente); // kp = 2/35 ; ki = 15/33 ; kd = 0
  
  erreur_precedente = erreur;

  // Conversion de la commande
  commande = (int)(255*commande/5);
  commande = commande>255 ? 255 : commande;
  commande = commande<0 ? 0 : commande;

  // Commande du PWM
  analogWrite(M1_pwm, commande);

  tick_M1=0;

  /************ MOTEUR 2*******************/

  // Calcul de la vitesse du moteur
  current_speed = tick_M2/(r_reduc*cad_ech*impulsion);

  // Calcul de l'erreur
  erreur = rep_capteur-current_speed; 
  somme_erreur+=erreur;

  // Calcul de la commande
  commande = 0.05714285714*erreur 
  + 0.45454545454*somme_erreur 
  + 0*(erreur-erreur_precedente); // kp = 2/35 ; ki = 15/33 ; kd = 0
  
  erreur_precedente = erreur;

  // Conversion de la commande
  commande = (int)(255*commande/5);
  commande = commande>255 ? 255 : commande;
  commande = commande<0 ? 0 : commande;

  // Commande du PWM
  analogWrite(M2_pwm, commande);

  tick_M1=0;


  sei();
  
}



void setup() {
  
  Serial.begin(9600);

  // Initialisation des pins moteur A
  pinMode(M1_sens_a, OUTPUT);// pin de direction 
  pinMode(M1_sens_b, OUTPUT);// pin de direction 
  pinMode(M1_pwm,OUTPUT); // pin du PWM (rapport cyclique)
  pinMode(M1_interruption,INPUT); // pin d'interruption

  // Initialisation des pins moteur B
  pinMode(M1_sens_a, OUTPUT);// pin de direction 
  pinMode(M1_sens_b, OUTPUT);// pin de direction 
  pinMode(M1_pwm,OUTPUT); // pin du PWM (rapport cyclique)
  pinMode(M1_interruption,INPUT); // pin d'interruption

  // Pins des capteurs
  pinMode(echo_a, INPUT);
  pinMode(trig_a, OUTPUT);
  pinMode(echo_b, INPUT);
  pinMode(trig_b, OUTPUT);

  // On fait tourner les moteurs dans un sens
  digitalWrite(M1_sens_a,HIGH);
  digitalWrite(M1_sens_b,LOW);
  digitalWrite(M2_sens_a,HIGH);
  digitalWrite(M2_sens_b,LOW);

  // on incrémente le nombre de tick dès qu'il y a un signal 
  //par le biais de la fonction comptage
  attachInterrupt(digitalPinToInterrupt(M1_interruption), comptage_M1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_interruption), comptage_M2, RISING);
  
  // TIMER 1 pour 100 hz
  cli(); 
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // on initialise un "buffer" pour le timer pour avoir la fréquence désiré
  OCR1A = 19999; // = 16000000 / (8 * 100) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 8 prescaler pour controler la vitesse du timer
  TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  // on allume le timer 1
  TIMSK1 |= (1 << OCIE1A);
  sei(); 
}



void loop() {
      /**** CAPTEUR A ****/
      // Envoi des ultrasons
      digitalWrite(trig_a, HIGH) ;
      delayMicroseconds (10) ;  
      digitalWrite(trig_a, LOW) ;

      //Reception des ultrasons
      temps = pulseIn(echo_a, HIGH, 10000);  
      distance_A = temps / 2.0 * vson ; // distance en mm
      
      /**** CAPTEUR B ****/
      // Envoi des ultrasons
      digitalWrite(trig_b, HIGH) ;
      delayMicroseconds (10) ;  
      digitalWrite(trig_b, LOW) ;

      //Reception des ultrasons
      temps = pulseIn(echo_b, HIGH, 10000);  
      distance_B = temps / 2.0 * vson ; // distance en mm

      // Affectation d'une consigne
      rep_capteur=consigne;
      rep_capteur = (distance_A < 500 || distance_B< 500 ) ? 1.9 : rep_capteur;
      rep_capteur = (distance_A < 400 || distance_B < 400)  ? 1.85 : rep_capteur;
      rep_capteur = (distance_A < 300 || distance_B < 300)  ? 1.75 : rep_capteur;
      rep_capteur = (distance_A < 200 || distance_B < 200)  ? 1.7 : rep_capteur;
      rep_capteur = (distance_A < 100 || distance_B < 100)  ? 0 : rep_capteur;

}