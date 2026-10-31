/*
 * =====================================================================
 *  ROBO CAR RACER - Desvio de obstaculos + manter-se na pista
 * =====================================================================
 *  Placa .......: ESP32 DevKit (ESP32-WROOM-32, 30 pinos)
 *  Motores .....: 4 motores DC (amarelos/azuis) em 2 pontes H L298N
 *  Sensores ....: 3x HC-SR04 (esquerda, centro, direita) no para-choque
 *  Servos ......: 3x micro servo 9g (direcao)
 *
 *  Como funciona:
 *   1. Le os 3 sensores ultrassonicos (um de cada vez, para nao haver
 *      interferencia entre eles).
 *   2. Calcula uma "direcao" entre -1 (todo para a esquerda) e +1
 *      (todo para a direita):
 *        - tenta ficar no MEIO da pista (compara esquerda x direita);
 *        - se algo aparece na frente, vira para o lado com mais espaco;
 *        - se uma parede lateral fica perto demais, se afasta dela.
 *   3. Suaviza a direcao (para nao ficar "tremendo") e aplica:
 *        - nos servos (rodas esterçam);
 *        - nos motores (lado de fora da curva gira mais rapido).
 *   4. Reduz a velocidade nas curvas e quando tem obstaculo perto.
 *   5. Se chegar MUITO perto de algo na frente, da re virando e
 *      depois segue.
 *
 *  NAO precisa de nenhuma biblioteca extra. Funciona no core ESP32
 *  do Arduino versao 2.x e 3.x.
 *
 *  Tudo que voce precisa ajustar esta na secao "CONFIGURACOES".
 * =====================================================================
 */

#include <Arduino.h>

// =====================================================================
//                         CONFIGURACOES - PINOS
// =====================================================================
// Se a sua ligacao for diferente, troque so os numeros aqui.
// (Os numeros sao os "Dxx" escritos na placa: D23 = 23, RX2 = 16, TX2 = 17,
//  VN = 39, VP = 36.)

// ---------- Ponte H 1 (FRENTE) : motor dianteiro ESQ (A) e DIR (B) ----------
const int PH1_ENA = 23;   // PWM motor dianteiro esquerdo
const int PH1_IN1 = 22;
const int PH1_IN2 = 21;
const int PH1_IN3 = 19;
const int PH1_IN4 = 18;
const int PH1_ENB = 5;    // PWM motor dianteiro direito

// ---------- Ponte H 2 (TRAS) : motor traseiro ESQ (A) e DIR (B) ----------
const int PH2_ENA = 4;    // PWM motor traseiro esquerdo
const int PH2_IN1 = 16;   // RX2
const int PH2_IN2 = 17;   // TX2
const int PH2_IN3 = 15;
const int PH2_IN4 = 2;
const int PH2_ENB = 26;   // PWM motor traseiro direito

// ---------- Sensores HC-SR04 ----------
// ECHO usa pinos 34, 35 e 39 (VN), que sao so de entrada - perfeitos p/ isso.
// ATENCAO: o ECHO do HC-SR04 sai com 5V. Use divisor de tensao
// (ex.: 1k em serie + 2k para o GND) para nao estressar o ESP32.
const int SENS_ESQ_TRIG    = 25;
const int SENS_ESQ_ECHO    = 39;  // VN
const int SENS_CENTRO_TRIG = 33;
const int SENS_CENTRO_ECHO = 34;
const int SENS_DIR_TRIG    = 32;
const int SENS_DIR_ECHO    = 35;

// ---------- Servos ----------
const int SERVO1_PIN = 13;   // servo da roda dianteira esquerda
const int SERVO2_PIN = 14;   // servo da roda dianteira direita
const int SERVO3_PIN = 27;   // terceiro servo (veja SERVO3_MODO abaixo)

// ---------- Botao de largada ----------
// Botao BOOT da placa (GPIO 0). Ligou o carrinho -> aperte BOOT -> ele larga.
const int  BOTAO_LARGADA   = 0;
const bool ESPERAR_BOTAO   = true;   // false = larga sozinho apos o atraso
const int  ATRASO_LARGADA_MS = 2000; // espera depois do botao/ligar

// =====================================================================
//                    CONFIGURACOES - COMPORTAMENTO
// =====================================================================

// ---------- Velocidade (0 a 255) ----------
int VEL_RETA      = 210;  // velocidade em reta
int VEL_CURVA     = 150;  // velocidade na curva mais fechada
int VEL_RE        = 170;  // velocidade da re de emergencia
int PWM_MINIMO    = 90;   // abaixo disso o motor nao vence o atrito

// Quanto os motores ajudam na curva (0 = nada, 1 = lado de dentro para).
// Valores > 1 fazem o lado de dentro girar para tras (curva bem fechada).
float GANHO_DIFERENCIAL = 0.7;

// ---------- Distancias (cm) ----------
float DIST_MAX           = 200; // leituras acima disso = "livre"
float DIST_FRENTE_ALERTA = 60;  // obstaculo na frente: comeca a desviar
float DIST_FRENTE_EMERG  = 15;  // muito perto: da re
float DIST_LATERAL_MIN   = 25;  // parede lateral perto: se afasta

// ---------- Ganhos da direcao ----------
float KP_CENTRALIZAR = 1.2;  // forca para ficar no meio da pista
float GANHO_FRENTE   = 1.0;  // forca do desvio do obstaculo frontal
float GANHO_PAREDE   = 0.8;  // forca para se afastar da parede lateral
float ZONA_MORTA     = 0.08; // erro pequeno = segue reto (evita zigue-zague)
float SUAVIZACAO     = 0.45; // 0..1 (menor = mais suave, maior = mais rapido)

// ---------- Servos ----------
// CENTRO: angulo em que a roda fica reta (ajuste fino de cada servo).
// SINAL:  +1 = normal, -1 = invertido (se a roda virar para o lado errado),
//          0 = fica sempre parado no centro.
// MAX:    quantos graus o servo vira para cada lado no maximo.
int   SERVO1_CENTRO = 90;  int SERVO1_SINAL = +1;
int   SERVO2_CENTRO = 90;  int SERVO2_SINAL = +1;
int   SERVO3_CENTRO = 90;
// SERVO3_MODO: 0 = parado no centro
//              1 = esterça junto com as dianteiras
//              2 = esterça ao contrario (direcao traseira, curva mais fechada)
int   SERVO3_MODO   = 0;
int   SERVO_MAX_GRAUS = 35;

// ---------- Motores invertidos ----------
// Se algum motor girar para tras quando deveria ir para frente, troque
// para true (ou inverta os fios dele no borne da ponte H).
bool INV_DIANT_ESQ = false;
bool INV_DIANT_DIR = false;
bool INV_TRAS_ESQ  = false;
bool INV_TRAS_DIR  = false;

// ---------- Debug ----------
// true = imprime distancias e direcao no Monitor Serial (115200).
const bool DEBUG = true;

// =====================================================================
//              DAQUI PARA BAIXO NAO PRECISA MEXER
// =====================================================================

// ----- PWM (compativel com core ESP32 2.x e 3.x) -----
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

const int PWM_MOTOR_FREQ = 1000; // L298N trabalha bem em ~1 kHz
const int PWM_MOTOR_RES  = 8;    // 0..255
const int PWM_SERVO_FREQ = 50;   // servo padrao: 50 Hz
const int PWM_SERVO_RES  = 16;   // 0..65535

// Canais: motores 0..3 (timers 0 e 1), servos 4..6 (timers 2 e 3)
const int CH_DIANT_ESQ = 0, CH_DIANT_DIR = 1, CH_TRAS_ESQ = 2, CH_TRAS_DIR = 3;
const int CH_SERVO1 = 4, CH_SERVO2 = 5, CH_SERVO3 = 6;

void pwmConfigurar(int pino, int canal, int freq, int res) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pino, freq, res, canal);
#else
  ledcSetup(canal, freq, res);
  ledcAttachPin(pino, canal);
#endif
}

void pwmEscrever(int pino, int canal, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)canal;
  ledcWrite(pino, duty);
#else
  (void)pino;
  ledcWrite(canal, duty);
#endif
}

// ----- Motores -----
struct Motor {
  int en, inA, inB, canal;
  bool invertido;
};

Motor motDiantEsq = {PH1_ENA, PH1_IN1, PH1_IN2, CH_DIANT_ESQ, false};
Motor motDiantDir = {PH1_ENB, PH1_IN3, PH1_IN4, CH_DIANT_DIR, false};
Motor motTrasEsq  = {PH2_ENA, PH2_IN1, PH2_IN2, CH_TRAS_ESQ,  false};
Motor motTrasDir  = {PH2_ENB, PH2_IN3, PH2_IN4, CH_TRAS_DIR,  false};

void motorIniciar(Motor &m) {
  pinMode(m.inA, OUTPUT);
  pinMode(m.inB, OUTPUT);
  digitalWrite(m.inA, LOW);
  digitalWrite(m.inB, LOW);
  pwmConfigurar(m.en, m.canal, PWM_MOTOR_FREQ, PWM_MOTOR_RES);
  pwmEscrever(m.en, m.canal, 0);
}

// vel: -255 (re total) .. +255 (frente total)
void motorAcionar(Motor &m, int vel) {
  if (m.invertido) vel = -vel;
  vel = constrain(vel, -255, 255);

  int modulo = abs(vel);
  if (modulo > 0 && modulo < PWM_MINIMO) modulo = PWM_MINIMO;

  if (vel > 0) {
    digitalWrite(m.inA, HIGH);
    digitalWrite(m.inB, LOW);
  } else if (vel < 0) {
    digitalWrite(m.inA, LOW);
    digitalWrite(m.inB, HIGH);
  } else {
    digitalWrite(m.inA, LOW);   // solto (desliza)
    digitalWrite(m.inB, LOW);
  }
  pwmEscrever(m.en, m.canal, modulo);
}

void motoresLados(int velEsq, int velDir) {
  motorAcionar(motDiantEsq, velEsq);
  motorAcionar(motTrasEsq,  velEsq);
  motorAcionar(motDiantDir, velDir);
  motorAcionar(motTrasDir,  velDir);
}

void motoresParar() { motoresLados(0, 0); }

// ----- Servos -----
void servoEscreverGraus(int pino, int canal, float graus) {
  graus = constrain(graus, 0.0f, 180.0f);
  // 0 graus = 500 us, 180 graus = 2500 us (periodo de 20000 us)
  float us = 500.0f + graus * (2000.0f / 180.0f);
  uint32_t duty = (uint32_t)(us * 65535.0f / 20000.0f);
  pwmEscrever(pino, canal, duty);
}

// direcao: -1 (esquerda) .. +1 (direita)
void servosDirecao(float direcao) {
  float graus = direcao * SERVO_MAX_GRAUS;
  servoEscreverGraus(SERVO1_PIN, CH_SERVO1, SERVO1_CENTRO + SERVO1_SINAL * graus);
  servoEscreverGraus(SERVO2_PIN, CH_SERVO2, SERVO2_CENTRO + SERVO2_SINAL * graus);

  float g3 = 0;
  if (SERVO3_MODO == 1) g3 = graus;
  if (SERVO3_MODO == 2) g3 = -graus;
  servoEscreverGraus(SERVO3_PIN, CH_SERVO3, SERVO3_CENTRO + g3);
}

// ----- Sensores -----
float distEsq = 0, distCentro = 0, distDir = 0;

float lerDistanciaCm(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Tempo maximo de espera = ida e volta de DIST_MAX (+ folga)
  unsigned long timeout = (unsigned long)(DIST_MAX * 58.0f) + 2000;
  unsigned long dur = pulseIn(echo, HIGH, timeout);
  if (dur == 0) return DIST_MAX;          // nada encontrado = livre

  float cm = dur / 58.0f;
  if (cm < 2) cm = 2;
  if (cm > DIST_MAX) cm = DIST_MAX;
  return cm;
}

// Filtro: ignora leituras "malucas" isoladas e suaviza
float filtrar(float anterior, float nova) {
  if (anterior <= 0) return nova;                  // primeira leitura
  // aproximacao rapida (seguranca), afastamento mais suave
  float alfa = (nova < anterior) ? 0.7f : 0.4f;
  return anterior + alfa * (nova - anterior);
}

void lerSensores() {
  distEsq    = filtrar(distEsq,    lerDistanciaCm(SENS_ESQ_TRIG,    SENS_ESQ_ECHO));
  delay(5);  // deixa o eco morrer antes do proximo sensor
  distCentro = filtrar(distCentro, lerDistanciaCm(SENS_CENTRO_TRIG, SENS_CENTRO_ECHO));
  delay(5);
  distDir    = filtrar(distDir,    lerDistanciaCm(SENS_DIR_TRIG,    SENS_DIR_ECHO));
  delay(5);
}

// ----- Controle -----
float direcaoSuave = 0;
unsigned long ultimoDebug = 0;

float limitar(float v, float mn, float mx) { return v < mn ? mn : (v > mx ? mx : v); }

void manobraEmergencia() {
  // Vira as rodas para o lado oposto ao que tem mais espaco e da re:
  // assim, quando voltar a andar para frente, o nariz aponta para o lado livre.
  float lado = (distDir >= distEsq) ? 1.0f : -1.0f;
  if (DEBUG) Serial.println(">>> EMERGENCIA: dando re");

  motoresParar();
  delay(80);
  servosDirecao(-lado);
  motoresLados(-VEL_RE, -VEL_RE);
  delay(350);
  motoresParar();
  delay(60);

  // Ja aponta para o lado livre
  direcaoSuave = lado;
  servosDirecao(lado);

  // Reinicia os filtros para ler o ambiente novo
  distEsq = distCentro = distDir = 0;
}

void controlar() {
  // Muito perto de algo na frente? Re!
  if (distCentro < DIST_FRENTE_EMERG) {
    manobraEmergencia();
    return;
  }

  // 1) Centralizar na pista: >0 = mais espaco a direita
  float soma = distEsq + distDir;
  float erro = (soma > 1) ? (distDir - distEsq) / soma : 0;  // -1..1
  if (fabs(erro) < ZONA_MORTA) erro = 0;
  float direcao = KP_CENTRALIZAR * erro;

  // 2) Obstaculo na frente: vira para o lado mais livre
  if (distCentro < DIST_FRENTE_ALERTA) {
    float urgencia = 1.0f - (distCentro - DIST_FRENTE_EMERG) /
                            (DIST_FRENTE_ALERTA - DIST_FRENTE_EMERG);
    urgencia = limitar(urgencia, 0, 1);
    float lado = (distDir >= distEsq) ? 1.0f : -1.0f;
    direcao += lado * urgencia * GANHO_FRENTE;
  }

  // 3) Parede lateral muito perto: se afasta dela
  if (distEsq < DIST_LATERAL_MIN)
    direcao += GANHO_PAREDE * (DIST_LATERAL_MIN - distEsq) / DIST_LATERAL_MIN;
  if (distDir < DIST_LATERAL_MIN)
    direcao -= GANHO_PAREDE * (DIST_LATERAL_MIN - distDir) / DIST_LATERAL_MIN;

  direcao = limitar(direcao, -1, 1);

  // Suaviza (curvas "bonitinhas", sem tremer)
  direcaoSuave += SUAVIZACAO * (direcao - direcaoSuave);

  // Velocidade: menor na curva e com obstaculo perto
  float fatorCurva  = fabs(direcaoSuave);
  float vel = VEL_RETA - (VEL_RETA - VEL_CURVA) * fatorCurva;
  if (distCentro < DIST_FRENTE_ALERTA) {
    float f = limitar((distCentro - DIST_FRENTE_EMERG) /
                      (DIST_FRENTE_ALERTA - DIST_FRENTE_EMERG), 0, 1);
    vel = VEL_CURVA + (vel - VEL_CURVA) * f;
  }

  // Diferencial: lado de fora da curva mais rapido
  float velEsq = vel * (1.0f + GANHO_DIFERENCIAL * direcaoSuave);
  float velDir = vel * (1.0f - GANHO_DIFERENCIAL * direcaoSuave);

  servosDirecao(direcaoSuave);
  motoresLados((int)velEsq, (int)velDir);

  if (DEBUG && millis() - ultimoDebug > 200) {
    ultimoDebug = millis();
    Serial.printf("E:%5.1f  C:%5.1f  D:%5.1f | dir:%+.2f | motE:%4d motD:%4d\n",
                  distEsq, distCentro, distDir, direcaoSuave,
                  (int)velEsq, (int)velDir);
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ROBO CAR RACER ===");

  motDiantEsq.invertido = INV_DIANT_ESQ;
  motDiantDir.invertido = INV_DIANT_DIR;
  motTrasEsq.invertido  = INV_TRAS_ESQ;
  motTrasDir.invertido  = INV_TRAS_DIR;

  motorIniciar(motDiantEsq);
  motorIniciar(motDiantDir);
  motorIniciar(motTrasEsq);
  motorIniciar(motTrasDir);

  pinMode(SENS_ESQ_TRIG, OUTPUT);    pinMode(SENS_ESQ_ECHO, INPUT);
  pinMode(SENS_CENTRO_TRIG, OUTPUT); pinMode(SENS_CENTRO_ECHO, INPUT);
  pinMode(SENS_DIR_TRIG, OUTPUT);    pinMode(SENS_DIR_ECHO, INPUT);
  digitalWrite(SENS_ESQ_TRIG, LOW);
  digitalWrite(SENS_CENTRO_TRIG, LOW);
  digitalWrite(SENS_DIR_TRIG, LOW);

  pwmConfigurar(SERVO1_PIN, CH_SERVO1, PWM_SERVO_FREQ, PWM_SERVO_RES);
  pwmConfigurar(SERVO2_PIN, CH_SERVO2, PWM_SERVO_FREQ, PWM_SERVO_RES);
  pwmConfigurar(SERVO3_PIN, CH_SERVO3, PWM_SERVO_FREQ, PWM_SERVO_RES);
  servosDirecao(0);  // rodas retas

  if (ESPERAR_BOTAO) {
    pinMode(BOTAO_LARGADA, INPUT_PULLUP);
    Serial.println("Aperte o botao BOOT para largar...");
    while (digitalRead(BOTAO_LARGADA) == HIGH) {
      // Enquanto espera, mostra as distancias (bom para testar sensores)
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
  distEsq = distCentro = distDir = 0;
}

void loop() {
  lerSensores();
  controlar();
}
