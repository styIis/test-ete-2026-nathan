/*
 * Robot Cachan - v5 : PID de vitesse + ODOMETRIE
 * Nucleo F446RE - upload SWD (STM32CubeProgrammer) OBLIGATOIRE
 *
 * ================= FAITS ETABLIS (calibres, ne plus refaire) =========
 * Roue GAUCHE : encodeur D4/D5, moteur D8/D10, 55 ticks/tour, signe -1
 * Roue DROITE : encodeur D3/D2, moteur D6/D7, 55 ticks/tour, signe -1
 * Decodage x1 (front montant de A) : indispensable, les fils encodeur
 *   et puissance partagent le meme cable -> le x4 perdait ~10 %.
 * Carte : SENS=HIGH -> PWM ACTIVE-BASSE. Aucune valeur n'est neutre
 *   dans les deux conventions -> voir drive().
 * Commande positive brute = RECULER -> inversion dans drive().
 * Modele vitesse : cmd ~= 27 * (tr/s) + 9, + FF_EXTRA_R cote droit
 * Vitesse max a vide ~7.6 tr/s (~1.2 m/s). Croisiere testee ~1.7-2.0 tr/s.
 *
 * RAPPEL MATERIEL - PCB ENCODEUR GAUCHE : jeu mecanique (soudure
 * d'ancrage cassee), cause un desalignement magnetique qui fait
 * decrocher le comptage de tics sous rotation. Stabilise avec des
 * gommes le 18/08/2026 (fix TEMPORAIRE) -> comptage verifie fiable
 * a 55 ticks/tour, ecarts G/D 0.9-1.5% sur plusieurs essais. Ne pas
 * bouger le PCB tant que la reparation definitive (ressoudage) n'est
 * pas faite a la reprise des cours. Le connecteur 6 broches lui-meme
 * n'est PAS en cause pour ce probleme-la (ecarte par test de
 * manipulation directe).
 *
 * RAPPEL MATERIEL - MOTEUR DROIT, RESISTANCE VARIABLE (probable
 * contact interne balais/collecteur, PAS le connecteur d'accouplement
 * driver<->moteur) : le 18/08/2026, resistance mesuree a 18 ohm contre
 * 5 ohm a gauche (mesure prise aux PATTES du connecteur cote moteur,
 * donc AVANT le point d'accouplement mâle-femelle avec le driver -->
 * le connecteur d'accouplement lui-meme n'est PAS teste par cette
 * mesure et reste une variable non verifiee). Causait un plafond de
 * vitesse (~1.93 tr/s max) et un delai de convergence de plusieurs
 * secondes cote droit. Apres de nombreuses rotations du moteur
 * pendant les tests, la resistance est redescendue a 5.3 ohm (quasi
 * identique au gauche) SANS intervention volontaire sur le cablage
 * -> plus coherent avec un contact balais/collecteur interne qui se
 * "rode" a l'usage qu'avec un probleme de connecteur externe.
 * Si le desequilibre G/D reapparait, RE-MESURER la resistance du
 * moteur droit au multimetre (aux pattes du connecteur, comme ici)
 * avant de retoucher FF_EXTRA_R.
 *
 * RAPPEL ALIMENTATION : la limite de courant de l'alimentation de
 * banc doit etre au MAXIMUM pendant les tests. Avec la limite basse,
 * le moteur droit (qui tirait plus de courant a cause de sa
 * resistance elevee) se faisait brider par l'alim -> ca ressemblait
 * a un blocage mecanique aleatoire des roues alors que ce n'en etait
 * pas un. Meme verification a faire cote alimentation batterie en
 * competition.
 *
 * ================= PROTOCOLE SERIE (115200) =========================
 *   V <g> <d>    consignes de roues, en tours/s      ex : V 2 2
 *   C <v> <w>    consigne robot : v en m/s, w en rad/s
 *                                                    ex : C 0.3 0
 *   S            stop
 *   R            remise a zero de l'odometrie
 *   O            affiche l'odometrie une fois
 *   T            active/desactive la telemetrie continue
 *   K            télémetrie 
 * ====================================================================
 */

#include <math.h>

// ---------------- Broches ----------------
const int MR_PWM = D6, MR_SENS = D7;   // roue DROITE
const int ML_PWM = D8, ML_SENS = D10;  // roue GAUCHE
const int ENC_L_A = D4, ENC_L_B = D5;
const int ENC_R_A = D3, ENC_R_B = D2;

// ---------------- Calibration figee ----------------
const float TPT_L = 55.0;  // ticks par tour, roue gauche
const float TPT_R = 55.0;  // ticks par tour, roue droite
const int SIGNE_L = -1;    // pour que "avancer" = positif
const int SIGNE_R = -1;

const float R_ROUE = 0.025;               // rayon de roue (m)
const float ENTRAXE = 0.23;               // distance entre roues (m)  <-- A CALIBRER
const float CIRCONF = 2.0 * PI * R_ROUE;  // 0.15708 m par tour

/*
 * MESURE PAR POUSSEE SUR 1.000 m, POST-STABILISATION PCB GAUCHE
 * (jeu mecanique du PCB encodeur gauche cale avec des gommes ;
 * l'ancienne valeur 180 ticks/m etait corrompue par le decrochage
 * magnetique du PCB desaligne, PAS par le connecteur) :
 *   gauche : ~328 ticks/m (moyenne sur 3 essais : 322, 330, 332)
 *   droite : ~330 ticks/m (moyenne sur 3 essais : 327, 333, 329)
 * Les deux roues donnent maintenant une circonference quasi identique,
 * coherent avec deux roues physiquement identiques (55 ticks/tour
 * chacune, confirme separement par comptage brut sur 10 tours a la
 * main : G=-549, D=-552, soit ~55 ticks/tour des deux cotes).
 * Validation croisee G/D sur 3 poussees de 1 m : ecarts 0.9-1.5%.
 * Validation aller-retour (3 essais) : retour a ~0 tick, +-6 ticks max.
 * Fixation temporaire aux gommes : reparation definitive de la
 * soudure du PCB gauche prevue a la reprise des cours.
 */
const float M_PAR_TICK_L = 0.003049;  // m/tick gauche (1/328) , post-stabilisation PCB, coherent avec droite
const float M_PAR_TICK_R = 0.003033;  // m/tick droite (1/329.7)

// ---------------- Regulation ----------------
const float FF_GAIN = 27.0;
const float FF_OFFSET = 9.0;
const float FF_EXTRA_R = 0;  // compensation moteur droit -- voir RAPPEL MATERIEL - MOTEUR DROIT ci-dessus
                             // Historique de reglage le 18/08/2026, teste avec V 1.7 1.7 (derive theta sur ~3s) :
                             //   FF_EXTRA_R=0  -> resistance 18 ohm -> -25.4 deg
                             //   FF_EXTRA_R=8  -> -16.9 deg (insuffisant)
                             //   FF_EXTRA_R=15 -> -7.9 deg (bon compromis avec resistance encore elevee)
                             //   FF_EXTRA_R=22-25 -> ~2-3 deg, PUIS resistance retombee a 5.3 ohm en cours de test
                             //   FF_EXTRA_R=5  -> -3.2 a +1.4 deg, valeur ACTUELLE, coherente avec resistance normale
                             // Si la resistance moteur droit remonte, remonter cette valeur en consequence
                             // (retester par paliers avec le protocole V 1.7 1.7 + relever theta a chaque essai).
const float KP = 4.0;
const float KI = 25.0;
const float INTEG_MAX = 3.0;
const int CMD_MAX = 255;
const int SLEW_MAX = 40;
const int DEADTIME_MS = 50;
const int ETABLI_MS = 200;
const unsigned long PERIODE = 100;  // ms -> 10 Hz

// Securite : coupe si aucune commande recue depuis ce delai.
const unsigned long TIMEOUT_MS = 3000;

// Flux machine pour le noeud ROS2 (bascule par la commande 'S')
bool fluxOdom = false;

// ---------------- Encodeurs (x1) ----------------
const unsigned long MIN_EDGE_US = 800;
volatile long ticksL = 0, ticksR = 0;
volatile unsigned long tLastL = 0, tLastR = 0;

void isrL() {
  unsigned long now = micros();
  if (now - tLastL < MIN_EDGE_US) return;
  tLastL = now;
  if (digitalRead(ENC_L_B)) ticksL--;
  else ticksL++;
}
void isrR() {
  unsigned long now = micros();
  if (now - tLastR < MIN_EDGE_US) return;
  tLastR = now;
  if (digitalRead(ENC_R_B)) ticksR--;
  else ticksR++;
}

// ---------------- Commande moteur ----------------
int sensL = LOW, sensR = LOW;

void drive(int pwmPin, int sensPin, int &sensMem, int cmd) {
  cmd = constrain(cmd, -255, 255);

  // Arret franc : le neutre fiable est SENS=LOW + 0.
  if (cmd == 0) {
    analogWrite(pwmPin, (sensMem == HIGH) ? 255 : 0);
    delay(5);
    digitalWrite(sensPin, LOW);
    analogWrite(pwmPin, 0);
    sensMem = LOW;
    return;
  }

  int c = -cmd;  // inversion carte
  int nouveauSens = (c >= 0) ? HIGH : LOW;

  if (nouveauSens != sensMem) {
    // Aucune valeur n'est neutre dans les deux conventions :
    //   SENS=LOW  -> 0 = arret,   255 = plein regime
    //   SENS=HIGH -> 255 = arret, 0   = plein regime (actif-bas)
    // On remet donc le neutre du NOUVEAU sens juste apres la bascule,
    // sans delai intermediaire, sinon a-coup pleine puissance.
    analogWrite(pwmPin, (sensMem == HIGH) ? 255 : 0);
    delay(DEADTIME_MS);
    digitalWrite(sensPin, nouveauSens);
    analogWrite(pwmPin, (nouveauSens == HIGH) ? 255 : 0);
    sensMem = nouveauSens;
    delay(ETABLI_MS);
  }

  if (nouveauSens == HIGH) analogWrite(pwmPin, 255 - c);
  else analogWrite(pwmPin, -c);
}
void driveL(int cmd) {
  drive(ML_PWM, ML_SENS, sensL, cmd);
}
void driveR(int cmd) {
  drive(MR_PWM, MR_SENS, sensR, cmd);
}
void stopAll() {
  digitalWrite(ML_SENS, LOW);
  analogWrite(ML_PWM, 0);
  sensL = LOW;
  digitalWrite(MR_SENS, LOW);
  analogWrite(MR_PWM, 0);
  sensR = LOW;
}

// ---------------- Etat de regulation ----------------
float integL = 0, integR = 0, cmdL = 0, cmdR = 0;
float consigneL = 0, consigneR = 0;  // tours/s
unsigned long derniereCommande = 0;
bool modeCalib = false;
bool telemetrie = true;

float boucle(float &integ, float &cmdPrec, float consigne, float mesure, float dt, float ffExtra = 0.0) {
  if (consigne == 0) {
    integ = 0;
    cmdPrec = 0;
    return 0;
  }
  float err = consigne - mesure;
  integ = constrain(integ + err * dt, -INTEG_MAX, INTEG_MAX);
  float ff = FF_GAIN * fabs(consigne) + FF_OFFSET + ffExtra;
  if (consigne < 0) ff = -ff;
  float out = constrain(ff + KP * err + KI * integ, -CMD_MAX, CMD_MAX);
  if (out > cmdPrec + SLEW_MAX) out = cmdPrec + SLEW_MAX;
  if (out < cmdPrec - SLEW_MAX) out = cmdPrec - SLEW_MAX;
  cmdPrec = out;
  return out;
}

// ---------------- Odometrie ----------------
// Pose du robot dans le repere de depart.
float odoX = 0, odoY = 0, odoTheta = 0;  // m, m, rad

/*
 * Modele differentiel :
 *   dCentre = (dGauche + dDroite) / 2
 *   dTheta  = (dDroite - dGauche) / ENTRAXE
 * On avance selon le cap MOYEN du pas (theta + dTheta/2) : c'est
 * l'integration du point milieu, nettement plus juste qu'utiliser
 * l'ancien cap, surtout en virage.
 */
void majOdometrie(long dTicksL, long dTicksR) {
  float dGauche = (SIGNE_L * dTicksL) * M_PAR_TICK_L;  // m
  float dDroite = (SIGNE_R * dTicksR) * M_PAR_TICK_R;  // m

  float dCentre = (dGauche + dDroite) * 0.5;
  float dTheta = (dDroite - dGauche) / ENTRAXE;

  float capMoyen = odoTheta + dTheta * 0.5;
  odoX += dCentre * cos(capMoyen);
  odoY += dCentre * sin(capMoyen);
  odoTheta += dTheta;

  // recentrage dans [-PI, PI]
  while (odoTheta > PI) odoTheta -= 2.0 * PI;
  while (odoTheta < -PI) odoTheta += 2.0 * PI;
}

void afficheOdometrie() {
  Serial.print("ODO  x=");
  Serial.print(odoX, 3);
  Serial.print(" m  y=");
  Serial.print(odoY, 3);
  Serial.print(" m  theta=");
  Serial.print(odoTheta * 180.0 / PI, 1);
  Serial.println(" deg");
}

/*
 * Cinematique inverse : consigne robot -> consignes de roues.
 *   v en m/s, w en rad/s
 *   vGauche = v - w * ENTRAXE/2   (m/s)  puis converti en tours/s
 */
void consigneRobot(float v, float w) {
  float vG = v - w * ENTRAXE * 0.5;
  float vD = v + w * ENTRAXE * 0.5;
  consigneL = vG / CIRCONF;
  consigneR = vD / CIRCONF;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(MR_PWM, OUTPUT);
  pinMode(MR_SENS, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(ML_SENS, OUTPUT);
  stopAll();

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);

  Serial.println();
  Serial.println("=== Robot Cachan v5 : PID + odometrie ===");
  Serial.println("V <g> <d> (tr/s) | C <v m/s> <w rad/s> | S | R | O | T");
  derniereCommande = millis();
}

// ---------------- Boucle ----------------
void loop() {
  static unsigned long tPrev = 0;
  static long lPrev = 0, rPrev = 0;

  // ---- lecture des commandes ----
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 0) {
      char c = s.charAt(0);
      if (c == 'V' || c == 'v') {
        modeCalib = false;
        int sp = s.indexOf(' ');
        int sp2 = s.indexOf(' ', sp + 1);
        if (sp > 0 && sp2 > 0) {
          consigneL = s.substring(sp + 1, sp2).toFloat();
          consigneR = s.substring(sp2 + 1).toFloat();
          derniereCommande = millis();
        }
      } else if (c == 'C' || c == 'c') {
        modeCalib = false;
        int sp = s.indexOf(' ');
        int sp2 = s.indexOf(' ', sp + 1);
        if (sp > 0 && sp2 > 0) {
          float v = s.substring(sp + 1, sp2).toFloat();
          float w = s.substring(sp2 + 1).toFloat();
          consigneRobot(v, w);
          derniereCommande = millis();
        }
      } else if (c == 'K' || c == 'k') {
        // Mode calibration : remet les ticks bruts a zero et les affiche.
        // Pousser ensuite le robot A LA MAIN, en ligne droite, sur 1.000 m.
        noInterrupts();
        ticksL = 0;
        ticksR = 0;
        interrupts();
        modeCalib = true;
        consigneL = consigneR = 0;
        stopAll();
        Serial.println("CALIB : ticks remis a zero.");
        Serial.println("Pousse le robot A LA MAIN, bien droit, sur 1.000 m.");
      } else if (c == 'R' || c == 'r') {
        odoX = odoY = odoTheta = 0;
        Serial.println("odometrie remise a zero");
      } else if (c == 'O' || c == 'o') {
        afficheOdometrie();
      } else if (c == 'T' || c == 't') {
        telemetrie = !telemetrie;
        Serial.print("telemetrie ");
        Serial.println(telemetrie ? "ON" : "OFF");
      } else if (c == 'S' || c == 's') {
        // 'S 1' active, 'S 0' coupe. 'S' seul bascule (pratique a la main).
        int sp = s.indexOf(' ');
        if (sp > 0) {
          fluxOdom = (s.substring(sp + 1).toInt() != 0);
        } else {
          fluxOdom = !fluxOdom;
        }
        if (fluxOdom) telemetrie = false;
        Serial.print("flux odom ");
        Serial.println(fluxOdom ? "ON" : "OFF");
      } else {
        consigneL = consigneR = 0;
        stopAll();
      }
    }
  }

  // ---- securite : coupure si plus de commande ----
  if ((consigneL != 0 || consigneR != 0) && millis() - derniereCommande > TIMEOUT_MS) {
    consigneL = consigneR = 0;
    Serial.println("TIMEOUT -> arret");
  }

  // ---- cycle de regulation + odometrie ----
  unsigned long now = millis();
  if (now - tPrev >= PERIODE) {
    float dt = (now - tPrev) / 1000.0;
    tPrev = now;

    noInterrupts();
    long l = ticksL, r = ticksR;
    interrupts();
    long dL = l - lPrev, dR = r - rPrev;
    lPrev = l;
    rPrev = r;

    // odometrie AVANT tout, sur les deltas bruts
    majOdometrie(dL, dR);

    // Vitesses exprimees en "tours/s d'une roue nominale" : on part de
    // la distance REELLE parcourue, divisee par la circonference
    // nominale. Ainsi deux consignes egales donnent deux vitesses
    // LINEAIRES egales -> le robot va droit meme si les deux roues
    // n'ont pas la meme resolution ni le meme rayon effectif.
    // Les gains et l'anticipation restent valables (memes unites).
    float vL = (SIGNE_L * dL * M_PAR_TICK_L / CIRCONF) / dt;
    float vR = (SIGNE_R * dR * M_PAR_TICK_R / CIRCONF) / dt;

    cmdL = boucle(integL, cmdL, consigneL, vL, dt);
    cmdR = boucle(integR, cmdR, consigneR, vR, dt, FF_EXTRA_R);
    driveL((int)cmdL);
    driveR((int)cmdR);

    if (modeCalib) {
      noInterrupts();
      long tl = ticksL, tr = ticksR;
      interrupts();
      Serial.print("CALIB  ticks bruts  G=");
      Serial.print(tl);
      Serial.print("  D=");
      Serial.print(tr);
      Serial.print("   -> si pousse sur 1.000 m :");
      if (tl != 0) {
        Serial.print("  M_PAR_TICK_L=");
        Serial.print(1.0 / fabs((float)tl), 6);
      }
      if (tr != 0) {
        Serial.print("  M_PAR_TICK_R=");
        Serial.print(1.0 / fabs((float)tr), 6);
      }
      Serial.println();
    } else if (telemetrie) {
      Serial.print("cons ");
      Serial.print(consigneL, 2);
      Serial.print("/");
      Serial.print(consigneR, 2);
      Serial.print("  mes ");
      Serial.print(vL, 2);
      Serial.print("/");
      Serial.print(vR, 2);
      Serial.print("  | x=");
      Serial.print(odoX, 3);
      Serial.print(" y=");
      Serial.print(odoY, 3);
      Serial.print(" th=");
      Serial.print(odoTheta * 180.0 / PI, 1);
      Serial.println(" deg");
    }
    if (fluxOdom) {
      float v = (vL + vR) * 0.5 * CIRCONF;      // m/s
      float w = (vR - vL) * CIRCONF / ENTRAXE;  // rad/s
      Serial.print("#O ");
      Serial.print(odoX, 4);
      Serial.print(' ');
      Serial.print(odoY, 4);
      Serial.print(' ');
      Serial.print(odoTheta, 4);
      Serial.print(' ');
      Serial.print(v, 4);
      Serial.print(' ');
      Serial.println(w, 4);
    }
  }
}
