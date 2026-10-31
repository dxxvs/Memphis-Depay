/*
 * =====================================================================
 *  ROBO CAR RACER - VERSAO DE TESTE (SEM SERVOS)
 * =====================================================================
 *  Placa .......: ESP32 DevKit (30 pinos)
 *  Motores .....: 4 motores DC em 2 pontes H L298N (rodas retas/travadas)
 *  Sensores ....: 3x HC-SR04 (esquerda, centro, direita)
 *
 *  Como funciona:
 *   - Caminho livre ............: anda RETO.
 *   - Parede perto de um lado ...: faz uma curva leve para o outro lado
 *                                  (para os motores do lado de dentro).
 *   - Obstaculo na frente .......: gira no lugar para o lado mais livre.
 *   - Obstaculo MUITO perto .....: para, da re, gira para o lado mais
 *                                  livre e volta a andar para frente.
 *
 *  Curva sem servo: um lado dos motores anda e o outro para (ou gira
 *  para tras), igual a um tanque de guerra.
 *
 *  Usa so os pinos IN1..IN4 das pontes H.
 *  Deixe os JUMPERS no ENA e ENB (motores sempre na velocidade maxima).
 *  NAO precisa de nenhuma biblioteca extra.
 * =====================================================================
 */

#include <Arduino.h>

// =====================================================================
//                         CONFIGURACOES - PINOS
// =====================================================================
// (Os numeros sao os "Dxx" escritos na placa: RX2 = 16, TX2 = 17, VN = 39.)

// ---------- Ponte H 1 (FRENTE) ----------
const int PH1_IN1 = 22;   // motor dianteiro esquerdo
const int PH1_IN2 = 21;   // motor dianteiro esquerdo
const int PH1_IN3 = 19;   // motor dianteiro direito
const int PH1_IN4 = 18;   // motor dianteiro direito

// ---------- Ponte H 2 (TRAS) ----------
const int PH2_IN1 = 16;   // RX2 - motor traseiro esquerdo
const int PH2_IN2 = 17;   // TX2 - motor traseiro esquerdo
const int PH2_IN3 = 15;   // motor traseiro direito
const int PH2_IN4 = 2;    // motor traseiro direito

// ---------- Sensores HC-SR04 ----------
// ATENCAO: o ECHO sai com 5V. O ideal e usar divisor de tensao
// (1k em serie + 2k para o GND).
const int SENS_ESQ_TRIG    = 25;
const int SENS_ESQ_ECHO    = 39;  // VN
const int SENS_CENTRO_TRIG = 33;
const int SENS_CENTRO_ECHO = 34;
const int SENS_DIR_TRIG    = 32;
const int SENS_DIR_ECHO    = 35;

// ---------- Botao de largada ----------
// Botao BOOT da placa. Liga o carrinho -> aperta BOOT -> ele larga.
const int  BOTAO_LARGADA     = 0;
const bool ESPERAR_BOTAO     = true;  // false = larga sozinho ao ligar
const int  ATRASO_LARGADA_MS = 2000;

// =====================================================================
//                    CONFIGURACOES - COMPORTAMENTO
// =====================================================================

// ---------- Distancias (cm) ----------
float DIST_MAX          = 200; // leitura acima disso = "livre"
float DIST_FRENTE_VIRAR = 40;  // obstaculo na frente: gira para desviar
float DIST_FRENTE_RE    = 20;  // muito perto: para, da re e gira
float DIST_LATERAL_MIN  = 20;  // parede lateral perto: curva para longe

// ---------- Tempos das manobras (ms) ----------
int TEMPO_PARADA_MS = 150;  // parada antes da re
int TEMPO_RE_MS     = 400;  // quanto tempo da re
int TEMPO_GIRO_MS   = 350;  // quanto tempo gira depois da re
// Se ele gira pouco, aumente TEMPO_GIRO_MS. Se gira demais, diminua.

// ---------- Motores invertidos ----------
// Se algum motor girar para tras quando deveria ir para frente, troque
// para true (ou inverta os fios dele no borne da ponte H).
bool INV_DIANT_ESQ = false;
bool INV_DIANT_DIR = false;
bool INV_TRAS_ESQ  = false;
bool INV_TRAS_DIR  = false;

// ---------- Debug ----------
// true = mostra distancias e o que ele esta fazendo no Monitor Serial (115200).
const bool DEBUG = true;

// =====================================================================
//              DAQUI PARA BAIXO NAO PRECISA MEXER
// =====================================================================

// ----- Motores -----
void motorIniciar(int inA, int inB) {
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  digitalWrite(inA, LOW);
  digitalWrite(inB, LOW);
}

// sentido: +1 = frente, -1 = re, 0 = parado
void motorAcionar(int inA, int inB, bool invertido, int sentido) {
  if (invertido) sentido = -sentido;
  if (sentido > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else if (sentido < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  }
}

// cada lado: +1 frente, -1 re, 0 parado
void motoresLados(int ladoEsq, int ladoDir) {
  motorAcionar(PH1_IN1, PH1_IN2, INV_DIANT_ESQ, ladoEsq);  // dianteiro esquerdo
  motorAcionar(PH2_IN1, PH2_IN2, INV_TRAS_ESQ,  ladoEsq);  // traseiro esquerdo
  motorAcionar(PH1_IN3, PH1_IN4, INV_DIANT_DIR, ladoDir);  // dianteiro direito
  motorAcionar(PH2_IN3, PH2_IN4, INV_TRAS_DIR,  ladoDir);  // traseiro direito
}

void frente()        { motoresLados( 1,  1); }
void re()            { motoresLados(-1, -1); }
void parar()         { motoresLados( 0,  0); }
void curvaDireita()  { motoresLados( 1,  0); }  // esquerdo anda, direito para
void curvaEsquerda() { motoresLados( 0,  1); }  // direito anda, esquerdo para
void girarDireita()  { motoresLados( 1, -1); }  // gira no lugar
void girarEsquerda() { motoresLados(-1,  1); }  // gira no lugar

// ----- Sensores -----
float distEsq = 0, distCentro = 0, distDir = 0;

float lerDistanciaCm(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long timeout = (unsigned long)(DIST_MAX * 58.0f) + 2000;
  unsigned long dur = pulseIn(echo, HIGH, timeout);
  if (dur == 0) return DIST_MAX;          // nada encontrado = livre

  float cm = dur / 58.0f;
  if (cm < 2) cm = 2;
  if (cm > DIST_MAX) cm = DIST_MAX;
  return cm;
}

void lerSensores() {
  distEsq = lerDistanciaCm(SENS_ESQ_TRIG, SENS_ESQ_ECHO);
  delay(5);  // espera o eco sumir antes do proximo sensor
  distCentro = lerDistanciaCm(SENS_CENTRO_TRIG, SENS_CENTRO_ECHO);
  delay(5);
  distDir = lerDistanciaCm(SENS_DIR_TRIG, SENS_DIR_ECHO);
  delay(5);
}

// ----- Controle -----
unsigned long ultimoDebug = 0;

void mostrar(const char *acao) {
  if (DEBUG && millis() - ultimoDebug > 200) {
    ultimoDebug = millis();
    Serial.printf("E:%5.1f  C:%5.1f  D:%5.1f  -> %s\n",
                  distEsq, distCentro, distDir, acao);
  }
}

void manobraRe() {
  bool direitaMaisLivre = (distDir >= distEsq);
  if (DEBUG) Serial.println(">>> MUITO PERTO: parando, dando re e girando");

  parar();
  delay(TEMPO_PARADA_MS);
  re();
  delay(TEMPO_RE_MS);
  parar();
  delay(100);

  if (direitaMaisLivre) girarDireita();
  else                  girarEsquerda();
  delay(TEMPO_GIRO_MS);
  parar();
  delay(50);
}

void controlar() {
  bool direitaMaisLivre = (distDir >= distEsq);

  if (distCentro < DIST_FRENTE_RE) {
    // 1) Muito perto: para, re, gira
    manobraRe();
  } else if (distCentro < DIST_FRENTE_VIRAR) {
    // 2) Obstaculo na frente: gira no lugar para o lado mais livre
    if (direitaMaisLivre) { girarDireita();  mostrar("girando p/ DIREITA"); }
    else                  { girarEsquerda(); mostrar("girando p/ ESQUERDA"); }
  } else if (distEsq < DIST_LATERAL_MIN && distDir < DIST_LATERAL_MIN) {
    // 3) Corredor estreito: vai para o lado mais livre
    if (direitaMaisLivre) { curvaDireita();  mostrar("estreito: curva DIREITA"); }
    else                  { curvaEsquerda(); mostrar("estreito: curva ESQUERDA"); }
  } else if (distEsq < DIST_LATERAL_MIN) {
    // 4) Parede na esquerda: curva para a direita
    curvaDireita();
    mostrar("parede ESQ: curva DIREITA");
  } else if (distDir < DIST_LATERAL_MIN) {
    // 5) Parede na direita: curva para a esquerda
    curvaEsquerda();
    mostrar("parede DIR: curva ESQUERDA");
  } else {
    // 6) Livre: reto
    frente();
    mostrar("RETO");
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ROBO CAR RACER (teste sem servo) ===");

  motorIniciar(PH1_IN1, PH1_IN2);
  motorIniciar(PH1_IN3, PH1_IN4);
  motorIniciar(PH2_IN1, PH2_IN2);
  motorIniciar(PH2_IN3, PH2_IN4);

  pinMode(SENS_ESQ_TRIG, OUTPUT);    pinMode(SENS_ESQ_ECHO, INPUT);
  pinMode(SENS_CENTRO_TRIG, OUTPUT); pinMode(SENS_CENTRO_ECHO, INPUT);
  pinMode(SENS_DIR_TRIG, OUTPUT);    pinMode(SENS_DIR_ECHO, INPUT);
  digitalWrite(SENS_ESQ_TRIG, LOW);
  digitalWrite(SENS_CENTRO_TRIG, LOW);
  digitalWrite(SENS_DIR_TRIG, LOW);

  if (ESPERAR_BOTAO) {
    pinMode(BOTAO_LARGADA, INPUT_PULLUP);
    Serial.println("Aperte o botao BOOT para largar...");
    while (digitalRead(BOTAO_LARGADA) == HIGH) {
      // Enquanto espera, mostra as distancias (para testar os sensores)
      lerSensores();
      if (DEBUG && millis() - ultimoDebug > 300) {
        ultimoDebug = millis();
        Serial.printf("[espera] E:%5.1f  C:%5.1f  D:%5.1f\n",
                      distEsq, distCentro, distDir);
      }
    }
  }

  Serial.println("Largando!");
  delay(ATRASO_LARGADA_MS);
}

void loop() {
  lerSensores();
  controlar();
}
