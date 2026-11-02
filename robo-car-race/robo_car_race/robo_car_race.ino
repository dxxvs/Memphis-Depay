/*
 * ============================================================================
 *  ROBÔ CAR RACE - Controle de 4 motores DC com ESP32 + 2x Ponte H L298N
 * ============================================================================
 *
 *  Placa .......: ESP32 DevKit (ESP-WROOM-32, 30 pinos, USB-C)
 *  Drivers .....: 2x L298N (ponte H dupla)
 *                   - L298N "FRENTE"   -> motor dianteiro esquerdo + dianteiro direito
 *                   - L298N "TRASEIRA" -> motor traseiro esquerdo  + traseiro direito
 *  Motores .....: 4x motor DC amarelo (TT, 3-6 V) com caixa de redução
 *  Alimentação .: bateria de 9 V
 *
 *  O que este código faz:
 *    - Controla os 4 motores de forma independente, com velocidade por PWM.
 *    - Liga o carrinho para FRENTE automaticamente após uma contagem inicial.
 *    - Acelera de forma suave (rampa) para não causar pico de corrente na
 *      bateria de 9 V (o que reiniciaria o ESP32 por "brownout").
 *    - Tem ajuste fino (trim) por motor para o carro andar reto.
 *    - Tem comandos pelo Monitor Serial (115200) para testar na bancada.
 *
 *  Compatível com o core ESP32 do Arduino versão 2.x e 3.x.
 *
 * ----------------------------------------------------------------------------
 *  LIGAÇÕES (ESP32 -> L298N)
 * ----------------------------------------------------------------------------
 *  IMPORTANTE: retire os jumpers ENA e ENB das DUAS pontes L298N, senão o
 *  PWM não funciona (o motor fica sempre em velocidade máxima).
 *
 *  L298N FRENTE (todos os pinos na fileira de cima da placa)
 *    ENA  -> D32   (PWM motor dianteiro esquerdo)
 *    IN1  -> D33
 *    IN2  -> D25
 *    IN3  -> D26
 *    IN4  -> D27
 *    ENB  -> D14   (PWM motor dianteiro direito)
 *    OUT1/OUT2 -> motor dianteiro esquerdo
 *    OUT3/OUT4 -> motor dianteiro direito
 *
 *  L298N TRASEIRA (todos os pinos na fileira de baixo da placa)
 *    ENA  -> D4    (PWM motor traseiro esquerdo)
 *    IN1  -> RX2 (GPIO16)
 *    IN2  -> TX2 (GPIO17)
 *    IN3  -> D18
 *    IN4  -> D19
 *    ENB  -> D21   (PWM motor traseiro direito)
 *    OUT1/OUT2 -> motor traseiro esquerdo
 *    OUT3/OUT4 -> motor traseiro direito
 *
 *  ALIMENTAÇÃO
 *    Bateria 9 V (+) -> chave liga/desliga -> borne "12V" das DUAS L298N
 *    Bateria 9 V (-) -> borne "GND" das DUAS L298N
 *    GND da L298N    -> GND do ESP32   (OBRIGATÓRIO: terra em comum!)
 *    5V da L298N FRENTE -> VIN do ESP32 (mantenha o jumper "5V-EN"/"12V"
 *                          da L298N colocado para o regulador funcionar)
 *    Não ligue o USB do PC e o 5V da L298N ao mesmo tempo no ESP32.
 *
 *  Pinos evitados de propósito:
 *    D34, D35, VP(36), VN(39) -> só entrada, não servem para motor
 *                                (mas servem para o ECHO dos ultrassônicos)
 *    D12, D15, D2, D5, D0     -> pinos de "boot" (strapping); podem impedir o
 *                                ESP32 de ligar/gravar se estiverem em nível errado
 *    TX0/RX0                  -> usados pelo USB/Monitor Serial
 *
 * ----------------------------------------------------------------------------
 *  SENSORES ULTRASSÔNICOS HC-SR04 (3 unidades)
 * ----------------------------------------------------------------------------
 *
 *            ESQ (20°)   FRENTE   DIR (20°)
 *                 \        |        /
 *                  \       |       /
 *                 [==]   [==]   [==]
 *                 +----------------+
 *                 |    CARRINHO    |
 *                 +----------------+
 *
 *    Sensor FRENTE ....: TRIG -> D13   ECHO -> D34
 *    Sensor ESQUERDA ..: TRIG -> D22   ECHO -> D35
 *    Sensor DIREITA ...: TRIG -> D23   ECHO -> VP (GPIO36)
 *    VCC dos 3 sensores -> 5V (mesmo 5V que vai no VIN do ESP32)
 *    GND dos 3 sensores -> GND comum
 *
 *  !!! ATENÇÃO: o ECHO do HC-SR04 sai com 5 V e o ESP32 aceita só 3,3 V !!!
 *  Use um divisor de tensão em CADA pino ECHO:
 *
 *     ECHO do sensor ---[ 1 kΩ ]---+---> pino do ESP32 (D34 / D35 / VP)
 *                                  |
 *                               [ 2 kΩ ]
 *                                  |
 *                                 GND
 *  (pode ser 10 kΩ + 20 kΩ, ou 1 kΩ + 2,2 kΩ: o importante é a proporção 1:2)
 *  O TRIG pode ir direto: 3,3 V do ESP32 é suficiente para disparar o sensor.
 *
 *  Os sensores são disparados UM DE CADA VEZ (para um não ouvir o eco do
 *  outro) e lidos por interrupção, então o programa nunca fica travado
 *  esperando o eco.
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
//  CONFIGURAÇÕES (ajuste aqui)
// ============================================================================

// Velocidade de corrida (0 a 255). 255 = máxima.
const uint8_t VELOCIDADE_CORRIDA = 200;

// Abaixo desse valor de PWM o motor amarelo costuma só "zumbir" sem girar.
// Qualquer velocidade > 0 pedida é elevada pelo menos até este valor.
const uint8_t PWM_MINIMO = 90;

// Tempo (ms) de espera depois de ligar, antes de sair andando.
const uint32_t ATRASO_LARGADA_MS = 3000;

// Rampa de aceleração: quanto o PWM muda a cada passo, e o tempo do passo.
// Ex.: passo 5 a cada 10 ms -> de 0 a 255 em ~0,5 s.
// Para DESACELERAR (frear/fazer curva) o passo é maior, porque reduzir a
// velocidade não força a bateria e o desvio precisa ser rápido.
const uint8_t  RAMPA_PASSO = 5;
const uint8_t  RAMPA_PASSO_FREIO = 40;
const uint32_t RAMPA_INTERVALO_MS = 10;

// Frequência e resolução do PWM. 1 kHz dá bom torque em baixa velocidade
// nos motores amarelos e funciona bem com a L298N.
const uint32_t PWM_FREQUENCIA = 1000;
const uint8_t  PWM_RESOLUCAO  = 8;      // 8 bits -> 0..255

// Se algum motor girar ao contrário, troque false por true (ou inverta os
// fios dele no borne da L298N).
const bool INVERTER_DE = false;   // dianteiro esquerdo
const bool INVERTER_DD = false;   // dianteiro direito
const bool INVERTER_TE = false;   // traseiro esquerdo
const bool INVERTER_TD = false;   // traseiro direito

// Ajuste fino (trim) em porcentagem, para compensar motores mais rápidos.
// Se o carro puxa para a ESQUERDA, o lado direito está mais rápido:
// diminua TRIM_DD e TRIM_TD (ex.: 95). E vice-versa.
const uint8_t TRIM_DE = 100;
const uint8_t TRIM_DD = 100;
const uint8_t TRIM_TE = 100;
const uint8_t TRIM_TD = 100;

// true  = liga o piloto automático (sensores) sozinho após ATRASO_LARGADA_MS
// false = fica parado esperando comandos do Monitor Serial ('r' = correr)
const bool LARGADA_AUTOMATICA = true;

// ----------------------------------------------------------------------------
//  PILOTO AUTOMÁTICO (sensores ultrassônicos) - distâncias em cm
// ----------------------------------------------------------------------------
// Calibre estes valores na pista! Ative a telemetria com 'u' no Monitor
// Serial para ver o que cada sensor está medindo.

// Sensor da FRENTE
const float DIST_LIVRE      = 120;  // acima disso: velocidade máxima
const float DIST_CURVA      = 60;   // abaixo disso: começa a curva forte
const float DIST_EMERGENCIA = 15;   // abaixo disso: dá ré e gira

// Sensores DIAGONAIS (20°)
const float DIST_DIAG_PERIGO = 25;  // abaixo disso: desvio forte para o outro lado
const float DIST_DIAG_LIMITE = 150; // acima disso a diagonal conta como "livre"

// Velocidades (0..255)
const uint8_t VEL_CURVA   = 140;    // velocidade base com obstáculo perto
const uint8_t VEL_MANOBRA = 170;    // velocidade da ré e do giro de emergência

// Ganhos da direção (quanto o carro vira)
const float KP_DIAGONAIS = 1.2;     // PWM por cm de diferença entre ESQ e DIR
const float KP_FRENTE    = 3.0;     // PWM por cm que a frente está abaixo de DIST_CURVA
const float GANHO_PERIGO = 120;     // PWM extra quando uma diagonal está em perigo
const float DIRECAO_MAX  = 255;     // limite da correção (255 permite girar no lugar)

// Manobra de emergência (quando a frente fica bloqueada)
const uint32_t TEMPO_RE_MS       = 300;  // tempo de ré
const uint32_t TEMPO_GIRO_MIN_MS = 150;  // tempo mínimo girando
const uint32_t TEMPO_GIRO_MAX_MS = 800;  // tempo máximo girando
const float    DIST_LIBERADA     = 45;   // frente livre o bastante para voltar a andar

// Leitura dos sensores
const uint32_t SLOT_SENSOR_MS = 20;      // tempo de cada sensor (3 sensores = 60 ms)
const float    DIST_MAX       = 300;     // "sem obstáculo" (cm)
const uint8_t  FALHAS_MAX     = 10;      // leituras sem resposta -> sensor com defeito

// ============================================================================
//  PINOS
// ============================================================================

// L298N FRENTE
const uint8_t PIN_ENA_F = 32;
const uint8_t PIN_IN1_F = 33;
const uint8_t PIN_IN2_F = 25;
const uint8_t PIN_IN3_F = 26;
const uint8_t PIN_IN4_F = 27;
const uint8_t PIN_ENB_F = 14;

// L298N TRASEIRA
const uint8_t PIN_ENA_T = 4;
const uint8_t PIN_IN1_T = 16;
const uint8_t PIN_IN2_T = 17;
const uint8_t PIN_IN3_T = 18;
const uint8_t PIN_IN4_T = 19;
const uint8_t PIN_ENB_T = 21;

// Ultrassônicos HC-SR04
const uint8_t PIN_TRIG_FRENTE = 13;
const uint8_t PIN_ECHO_FRENTE = 34;
const uint8_t PIN_TRIG_ESQ    = 22;
const uint8_t PIN_ECHO_ESQ    = 35;
const uint8_t PIN_TRIG_DIR    = 23;
const uint8_t PIN_ECHO_DIR    = 36;   // VP

// ============================================================================
//  PWM compatível com core ESP32 2.x e 3.x
// ============================================================================

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  // Core 3.x: o PWM é endereçado pelo próprio pino.
  static void pwmConfigurar(uint8_t pino, uint8_t /*canal*/) {
    ledcAttach(pino, PWM_FREQUENCIA, PWM_RESOLUCAO);
  }
  static void pwmEscrever(uint8_t pino, uint8_t /*canal*/, uint32_t duty) {
    ledcWrite(pino, duty);
  }
#else
  // Core 2.x: o PWM é endereçado por canal (0..15).
  static void pwmConfigurar(uint8_t pino, uint8_t canal) {
    ledcSetup(canal, PWM_FREQUENCIA, PWM_RESOLUCAO);
    ledcAttachPin(pino, canal);
  }
  static void pwmEscrever(uint8_t /*pino*/, uint8_t canal, uint32_t duty) {
    ledcWrite(canal, duty);
  }
#endif

// ============================================================================
//  MOTOR
// ============================================================================

struct Motor {
  const char* nome;
  uint8_t pinoEn;     // pino de PWM (ENA/ENB)
  uint8_t pinoA;      // IN1 ou IN3
  uint8_t pinoB;      // IN2 ou IN4
  uint8_t canalPwm;   // usado só no core 2.x
  bool    invertido;
  uint8_t trim;       // %

  int16_t atual;      // velocidade aplicada agora (-255..255)
  int16_t alvo;       // velocidade desejada      (-255..255)

  void iniciar() {
    pinMode(pinoA, OUTPUT);
    pinMode(pinoB, OUTPUT);
    digitalWrite(pinoA, LOW);
    digitalWrite(pinoB, LOW);
    pwmConfigurar(pinoEn, canalPwm);
    pwmEscrever(pinoEn, canalPwm, 0);
    atual = 0;
    alvo  = 0;
  }

  // Aplica imediatamente uma velocidade -255..255 nos pinos.
  void aplicar(int16_t vel) {
    vel = constrain(vel, -255, 255);
    if (invertido) vel = -vel;

    if (vel > 0) {
      digitalWrite(pinoA, HIGH);
      digitalWrite(pinoB, LOW);
    } else if (vel < 0) {
      digitalWrite(pinoA, LOW);
      digitalWrite(pinoB, HIGH);
    } else {
      // Parado: os dois em LOW = motor solto (para por inércia).
      digitalWrite(pinoA, LOW);
      digitalWrite(pinoB, LOW);
    }

    uint32_t duty = (uint32_t)abs(vel) * trim / 100;
    if (duty > 255) duty = 255;
    pwmEscrever(pinoEn, canalPwm, duty);
  }

  // Avança um passo da rampa em direção ao alvo.
  void atualizarRampa() {
    if (atual == alvo) return;

    // Se vai inverter o sentido, passa pelo zero primeiro.
    int16_t destino = alvo;
    if ((atual > 0 && alvo < 0) || (atual < 0 && alvo > 0)) destino = 0;

    // Acelerando (|velocidade| aumentando) -> passo suave.
    // Desacelerando (|velocidade| diminuindo) -> passo rápido.
    bool acelerando = abs(destino) > abs(atual);
    int16_t passo = acelerando ? RAMPA_PASSO : RAMPA_PASSO_FREIO;

    if (atual < destino) atual = min<int16_t>(atual + passo, destino);
    else                 atual = max<int16_t>(atual - passo, destino);

    // Na partida, pula direto para o PWM mínimo (abaixo dele o motor não gira).
    if (atual > 0 && atual < PWM_MINIMO && destino >= PWM_MINIMO)  atual = PWM_MINIMO;
    if (atual < 0 && atual > -PWM_MINIMO && destino <= -PWM_MINIMO) atual = -PWM_MINIMO;

    aplicar(atual);
  }

  // Freio imediato, sem rampa.
  void pararJa() {
    alvo = 0;
    atual = 0;
    aplicar(0);
  }
};

Motor motorDE = {"Dianteiro Esq", PIN_ENA_F, PIN_IN1_F, PIN_IN2_F, 0, INVERTER_DE, TRIM_DE, 0, 0};
Motor motorDD = {"Dianteiro Dir", PIN_ENB_F, PIN_IN3_F, PIN_IN4_F, 1, INVERTER_DD, TRIM_DD, 0, 0};
Motor motorTE = {"Traseiro Esq",  PIN_ENA_T, PIN_IN1_T, PIN_IN2_T, 2, INVERTER_TE, TRIM_TE, 0, 0};
Motor motorTD = {"Traseiro Dir",  PIN_ENB_T, PIN_IN3_T, PIN_IN4_T, 3, INVERTER_TD, TRIM_TD, 0, 0};

Motor* motores[] = {&motorDE, &motorDD, &motorTE, &motorTD};
const uint8_t NUM_MOTORES = sizeof(motores) / sizeof(motores[0]);

// ============================================================================
//  FUNÇÕES DE MOVIMENTO
// ============================================================================

// Ajusta a velocidade pedida: 0 continua 0; qualquer outro valor fica entre
// PWM_MINIMO e 255 (mantendo o sinal).
static int16_t ajustarVelocidade(int16_t v) {
  if (v == 0) return 0;
  int16_t m = constrain(abs(v), (int)PWM_MINIMO, 255);
  return v > 0 ? m : -m;
}

// Define a velocidade do lado esquerdo e do lado direito (-255..255).
// Positivo = frente, negativo = ré. A mudança acontece com rampa suave.
void definirLados(int16_t esquerda, int16_t direita) {
  esquerda = ajustarVelocidade(esquerda);
  direita  = ajustarVelocidade(direita);
  motorDE.alvo = esquerda;
  motorTE.alvo = esquerda;
  motorDD.alvo = direita;
  motorTD.alvo = direita;
}

void frente(uint8_t vel)        { definirLados( vel,  vel); }
void re(uint8_t vel)            { definirLados(-vel, -vel); }
void girarEsquerda(uint8_t vel) { definirLados(-vel,  vel); }  // gira no lugar
void girarDireita(uint8_t vel)  { definirLados( vel, -vel); }  // gira no lugar

// Curva suave: o lado de dentro anda mais devagar (fator 0..100 %).
void curvaEsquerda(uint8_t vel, uint8_t fatorDentro = 40) {
  definirLados((int16_t)vel * fatorDentro / 100, vel);
}
void curvaDireita(uint8_t vel, uint8_t fatorDentro = 40) {
  definirLados(vel, (int16_t)vel * fatorDentro / 100);
}

void parar()   { definirLados(0, 0); }                 // para com rampa
void pararJa() { for (auto m : motores) m->pararJa(); } // para na hora

// Chamar sempre no loop(): executa a rampa sem travar o programa.
void atualizarMotores() {
  static uint32_t ultimo = 0;
  uint32_t agora = millis();
  if (agora - ultimo < RAMPA_INTERVALO_MS) return;
  ultimo = agora;
  for (auto m : motores) m->atualizarRampa();
}


// Velocidade de cruzeiro atual (começa em VELOCIDADE_CORRIDA; muda com 0..9)
uint8_t velocidadeAtual = VELOCIDADE_CORRIDA;

// ============================================================================
//  SENSORES ULTRASSÔNICOS HC-SR04
// ============================================================================

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

struct Ultrassom {
  const char* nome;
  uint8_t trig;
  uint8_t echo;

  // Preenchidos pela interrupção do pino ECHO
  volatile uint32_t tSubida;   // micros() quando o ECHO subiu
  volatile uint32_t duracao;   // largura do pulso de ECHO (µs)
  volatile bool     subiu;     // ECHO subiu depois do disparo
  volatile bool     novo;      // pulso completo recebido

  float   historico[3];        // últimas 3 leituras (filtro de mediana)
  uint8_t idxHist;
  float   distancia;           // distância filtrada (cm)
  uint8_t falhas;              // leituras seguidas sem nenhuma resposta
};

Ultrassom sensorFrente = {"FRENTE", PIN_TRIG_FRENTE, PIN_ECHO_FRENTE, 0, 0, false, false, {0, 0, 0}, 0, 0, 0};
Ultrassom sensorEsq    = {"ESQ",    PIN_TRIG_ESQ,    PIN_ECHO_ESQ,    0, 0, false, false, {0, 0, 0}, 0, 0, 0};
Ultrassom sensorDir    = {"DIR",    PIN_TRIG_DIR,    PIN_ECHO_DIR,    0, 0, false, false, {0, 0, 0}, 0, 0, 0};

Ultrassom* sensores[] = {&sensorFrente, &sensorEsq, &sensorDir};
const uint8_t NUM_SENSORES = sizeof(sensores) / sizeof(sensores[0]);

// Interrupção: mede o tempo em que o ECHO fica em nível alto.
void IRAM_ATTR isrEcho(void* arg) {
  Ultrassom* s = (Ultrassom*)arg;
  uint32_t agora = micros();
  if (digitalRead(s->echo)) {
    s->tSubida = agora;
    s->subiu = true;
  } else if (s->subiu) {
    s->duracao = agora - s->tSubida;
    s->novo = true;
  }
}

static float mediana3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

void iniciarSensores() {
  for (auto s : sensores) {
    pinMode(s->trig, OUTPUT);
    digitalWrite(s->trig, LOW);
    pinMode(s->echo, INPUT);
    for (auto& h : s->historico) h = DIST_MAX;
    s->idxHist = 0;
    s->distancia = DIST_MAX;
    s->falhas = 0;
    s->subiu = false;
    s->novo = false;
    attachInterruptArg(digitalPinToInterrupt(s->echo), isrEcho, s, CHANGE);
  }
}

static void dispararSensor(Ultrassom& s) {
  s.subiu = false;
  s.novo = false;
  digitalWrite(s.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(s.trig, LOW);
}

// Chamado no fim do "slot" do sensor: transforma o pulso em centímetros.
static void finalizarLeitura(Ultrassom& s) {
  float cm;
  if (s.novo) {
    cm = s.duracao / 58.0f;              // tempo (µs) -> distância (cm)
    if (cm < 2) cm = 2;                  // HC-SR04 não mede menos que ~2 cm
    if (cm > DIST_MAX) cm = DIST_MAX;
    s.falhas = 0;
  } else if (s.subiu) {
    cm = DIST_MAX;                       // eco não voltou a tempo: caminho livre
    s.falhas = 0;
  } else {
    cm = DIST_MAX;                       // sensor nem respondeu: fio solto?
    if (s.falhas < 255) s.falhas++;
  }
  s.historico[s.idxHist] = cm;
  s.idxHist = (s.idxHist + 1) % 3;
  s.distancia = mediana3(s.historico[0], s.historico[1], s.historico[2]);
}

// Chamar sempre no loop(): dispara um sensor por vez, sem travar o programa.
// Retorna true quando uma leitura nova ficou pronta.
bool atualizarSensores() {
  static uint8_t  atual = 0;
  static uint32_t tDisparo = 0;
  static bool     primeiro = true;

  uint32_t agora = millis();
  if (!primeiro && agora - tDisparo < SLOT_SENSOR_MS) return false;

  bool leituraNova = false;
  if (!primeiro) {
    finalizarLeitura(*sensores[atual]);
    atual = (atual + 1) % NUM_SENSORES;
    leituraNova = true;
  }
  primeiro = false;

  dispararSensor(*sensores[atual]);
  tDisparo = agora;
  return leituraNova;
}

bool sensorComDefeito() {
  for (auto s : sensores) if (s->falhas >= FALHAS_MAX) return true;
  return false;
}

// ============================================================================
//  PILOTO AUTOMÁTICO
// ============================================================================
//
//  Ideia: o carro tenta sempre ir para o lado que tem MAIS espaço.
//   - As duas diagonais (20°) são comparadas: se a esquerda tem mais espaço
//     que a direita, o carro vira para a esquerda, e vice-versa. Isso mantém
//     o carro no meio da pista e faz ele acompanhar as curvas.
//   - A frente controla a velocidade: livre = rápido, perto = devagar e
//     curva mais forte para o lado livre.
//   - Diagonal muito perto (obstáculo/parede de lado) = desvio forte.
//   - Frente bloqueada = ré + giro para o lado mais livre.

enum EstadoPiloto { SEGUINDO, DANDO_RE, GIRANDO };
const char* NOMES_ESTADO[] = {"SEGUINDO", "RE", "GIRANDO"};

bool         modoAutonomo = false;
EstadoPiloto estado = SEGUINDO;
uint32_t     tEstado = 0;
bool         giroParaEsquerda = true;
uint32_t     tUltimaEmergencia = 0;
int16_t      saidaEsq = 0, saidaDir = 0;   // última saída (para telemetria)

void comandarLados(int16_t esq, int16_t dir) {
  saidaEsq = esq;
  saidaDir = dir;
  definirLados(esq, dir);
}

void iniciarPiloto() {
  modoAutonomo = true;
  estado = SEGUINDO;
  tEstado = millis();
  Serial.println(F("PILOTO AUTOMATICO LIGADO"));
}

void pararPiloto() {
  modoAutonomo = false;
  estado = SEGUINDO;
}

void pilotoAutomatico() {
  uint32_t agora = millis();
  float f = sensorFrente.distancia;
  float e = sensorEsq.distancia;
  float d = sensorDir.distancia;

  if (sensorComDefeito()) {
    pararJa();
    pararPiloto();
    Serial.println(F("ERRO: um sensor nao responde. Confira TRIG/ECHO/VCC/GND."));
    for (auto s : sensores)
      if (s->falhas >= FALHAS_MAX) Serial.printf("  -> sensor %s\n", s->nome);
    return;
  }

  switch (estado) {
    case SEGUINDO: {
      // ---- Emergência: frente bloqueada, ou as duas diagonais fechadas ----
      if (f < DIST_EMERGENCIA || (e < DIST_EMERGENCIA && d < DIST_EMERGENCIA)) {
        bool novoLado = e > d;   // gira para o lado com mais espaço
        // Se bateu de novo logo em seguida, tenta o outro lado.
        if (agora - tUltimaEmergencia < 1500) novoLado = !giroParaEsquerda;
        giroParaEsquerda = novoLado;
        tUltimaEmergencia = agora;
        estado = DANDO_RE;
        tEstado = agora;
        comandarLados(-VEL_MANOBRA, -VEL_MANOBRA);
        return;
      }

      // ---- Velocidade base pela distância da frente ----
      float base;
      if (f >= DIST_LIVRE) {
        base = velocidadeAtual;
      } else {
        float k = (f - DIST_EMERGENCIA) / (DIST_LIVRE - DIST_EMERGENCIA);
        base = VEL_CURVA + k * ((float)velocidadeAtual - VEL_CURVA);
      }

      // ---- Direção: positivo = vira para a ESQUERDA ----
      float el = min(e, DIST_DIAG_LIMITE);
      float dl = min(d, DIST_DIAG_LIMITE);
      float erro = el - dl;                      // > 0: mais espaço à esquerda
      float direcao = KP_DIAGONAIS * erro;

      // Obstáculo/curva à frente: reforça a curva para o lado mais livre.
      if (f < DIST_CURVA) {
        float reforco = (DIST_CURVA - f) * KP_FRENTE;
        direcao += (erro >= 0) ? reforco : -reforco;
      }

      // Diagonal em perigo: foge para o outro lado.
      if (e < DIST_DIAG_PERIGO) direcao -= GANHO_PERIGO * (1 - e / DIST_DIAG_PERIGO) + GANHO_PERIGO / 2;
      if (d < DIST_DIAG_PERIGO) direcao += GANHO_PERIGO * (1 - d / DIST_DIAG_PERIGO) + GANHO_PERIGO / 2;

      direcao = constrain(direcao, -DIRECAO_MAX, DIRECAO_MAX);

      // Virar à esquerda = lado esquerdo mais devagar, direito mais rápido.
      int16_t ve = constrain((int)(base - direcao), -255, 255);
      int16_t vd = constrain((int)(base + direcao), -255, 255);
      comandarLados(ve, vd);
      break;
    }

    case DANDO_RE:
      if (agora - tEstado >= TEMPO_RE_MS) {
        estado = GIRANDO;
        tEstado = agora;
        if (giroParaEsquerda) comandarLados(-VEL_MANOBRA,  VEL_MANOBRA);
        else                  comandarLados( VEL_MANOBRA, -VEL_MANOBRA);
      }
      break;

    case GIRANDO: {
      uint32_t t = agora - tEstado;
      if ((t >= TEMPO_GIRO_MIN_MS && f > DIST_LIBERADA) || t >= TEMPO_GIRO_MAX_MS) {
        estado = SEGUINDO;
        tEstado = agora;
      }
      break;
    }
  }
}

// ============================================================================
//  COMANDOS PELO MONITOR SERIAL (115200 baud) - para testes
// ============================================================================
//   r = CORRER (liga o piloto automático com os sensores)
//   w = frente      s = ré         a = girar esq.   d = girar dir.
//   q = curva esq.  e = curva dir. x = parar (rampa) espaço = parar já
//   0..9 = velocidade (0 = 0 %, 9 = 100 %)
//   u = liga/desliga telemetria dos sensores
//   t = teste de cada motor, um por vez
//   ? = ajuda
//  Qualquer comando de movimento manual desliga o piloto automático.

char    ultimoMovimento = 'x';
bool    comandoRecebido = false;   // qualquer comando cancela a largada automática
bool    telemetria = false;

void imprimirAjuda() {
  Serial.println();
  Serial.println(F("=== ROBO CAR RACE - comandos ==="));
  Serial.println(F(" r CORRER (piloto automatico com sensores)"));
  Serial.println(F(" w frente | s re | a girar esq | d girar dir"));
  Serial.println(F(" q curva esq | e curva dir | x parar | espaco parar ja"));
  Serial.println(F(" 0..9 velocidade | u telemetria | t testar motores | ? ajuda"));
  Serial.printf("Velocidade atual: %u/255\n", velocidadeAtual);
}

void executarMovimento(char c) {
  switch (c) {
    case 'w': frente(velocidadeAtual);        break;
    case 's': re(velocidadeAtual);            break;
    case 'a': girarEsquerda(velocidadeAtual); break;
    case 'd': girarDireita(velocidadeAtual);  break;
    case 'q': curvaEsquerda(velocidadeAtual); break;
    case 'e': curvaDireita(velocidadeAtual);  break;
    case 'x': parar();                        break;
    default: return;
  }
  ultimoMovimento = c;
}

// Gira cada motor para frente por 1 s, um de cada vez. Útil para conferir
// a fiação e o sentido de rotação (todos devem empurrar o carro para frente).
void testarMotores() {
  pararJa();
  for (auto m : motores) {
    Serial.printf("Testando: %s\n", m->nome);
    m->aplicar(VELOCIDADE_CORRIDA);
    delay(1000);
    m->aplicar(0);
    delay(400);
  }
  Serial.println(F("Teste concluido."));
}

void lerSerial() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == '\n' || c == '\r' || c == 0) continue;
    comandoRecebido = true;

    if (c >= '0' && c <= '9') {
      velocidadeAtual = (uint8_t)((c - '0') * 255 / 9);
      Serial.printf("Velocidade: %u/255\n", velocidadeAtual);
      if (!modoAutonomo) executarMovimento(ultimoMovimento);  // reaplica
    } else if (c == 'r') {
      iniciarPiloto();
    } else if (c == 'u') {
      telemetria = !telemetria;
      Serial.printf("Telemetria: %s\n", telemetria ? "ligada" : "desligada");
    } else if (c == ' ') {
      pararPiloto();
      pararJa();
      ultimoMovimento = 'x';
      Serial.println(F("PARADA IMEDIATA"));
    } else if (c == 't') {
      pararPiloto();
      testarMotores();
      ultimoMovimento = 'x';
    } else if (c == '?' || c == 'h') {
      imprimirAjuda();
    } else if (strchr("wsadqex", c)) {
      pararPiloto();
      executarMovimento(c);
      Serial.printf("Movimento manual: %c\n", c);
    }
  }
}

void imprimirTelemetria() {
  static uint32_t ultimo = 0;
  if (!telemetria || millis() - ultimo < 200) return;
  ultimo = millis();
  Serial.printf("ESQ %5.0f | FRENTE %5.0f | DIR %5.0f cm | %s | motores E %4d D %4d\n",
                sensorEsq.distancia, sensorFrente.distancia, sensorDir.distancia,
                modoAutonomo ? NOMES_ESTADO[estado] : "MANUAL",
                modoAutonomo ? saidaEsq : 0, modoAutonomo ? saidaDir : 0);
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================

uint32_t tLigado = 0;
bool     largadaPendente = false;

void setup() {
  Serial.begin(115200);

  for (auto m : motores) m->iniciar();
  iniciarSensores();

  imprimirAjuda();

  tLigado = millis();
  largadaPendente = LARGADA_AUTOMATICA;
  if (largadaPendente)
    Serial.printf("Largada automatica em %lu ms... (qualquer comando cancela)\n",
                  (unsigned long)ATRASO_LARGADA_MS);
}

void loop() {
  lerSerial();

  // Largada automática (os sensores já vão medindo durante a contagem).
  if (largadaPendente) {
    if (comandoRecebido) {
      largadaPendente = false;
      Serial.println(F("Largada automatica cancelada (comando recebido)."));
    } else if (millis() - tLigado >= ATRASO_LARGADA_MS) {
      largadaPendente = false;
      Serial.println(F("VAI!"));
      iniciarPiloto();
    }
  }

  // Cada leitura nova de sensor = uma decisão nova do piloto.
  if (atualizarSensores() && modoAutonomo) pilotoAutomatico();
  // Estados com tempo (ré/giro) também são conferidos fora das leituras.
  else if (modoAutonomo && estado != SEGUINDO) pilotoAutomatico();

  atualizarMotores();
  imprimirTelemetria();
}
