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
 *    D12, D15, D2, D5, D0     -> pinos de "boot" (strapping); podem impedir o
 *                                ESP32 de ligar/gravar se estiverem em nível errado
 *    TX0/RX0                  -> usados pelo USB/Monitor Serial
 *  Ficam livres para sensores (ex.: ultrassônico, infravermelho):
 *    D13, D22, D23, D34, D35, VP, VN
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
const uint8_t  RAMPA_PASSO = 5;
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

// true  = sai andando para frente sozinho após ATRASO_LARGADA_MS
// false = fica parado esperando comandos do Monitor Serial
const bool LARGADA_AUTOMATICA = true;

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

    if (atual < destino) atual = min<int16_t>(atual + RAMPA_PASSO, destino);
    else                 atual = max<int16_t>(atual - RAMPA_PASSO, destino);

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

// ============================================================================
//  COMANDOS PELO MONITOR SERIAL (115200 baud) - para testes
// ============================================================================
//   w = frente      s = ré         a = girar esq.   d = girar dir.
//   q = curva esq.  e = curva dir. x = parar (rampa) espaço = parar já
//   0..9 = velocidade (0 = 0 %, 9 = 100 %)
//   t = teste de cada motor, um por vez
//   ? = ajuda

uint8_t velocidadeAtual = VELOCIDADE_CORRIDA;
char    ultimoMovimento = 'x';
bool    comandoRecebido = false;   // qualquer comando cancela a largada automática

void imprimirAjuda() {
  Serial.println();
  Serial.println(F("=== ROBO CAR RACE - comandos ==="));
  Serial.println(F(" w frente | s re | a girar esq | d girar dir"));
  Serial.println(F(" q curva esq | e curva dir | x parar | espaco parar ja"));
  Serial.println(F(" 0..9 velocidade | t testar motores | ? ajuda"));
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
      executarMovimento(ultimoMovimento);   // reaplica o movimento atual
    } else if (c == ' ') {
      pararJa();
      ultimoMovimento = 'x';
      Serial.println(F("PARADA IMEDIATA"));
    } else if (c == 't') {
      testarMotores();
      ultimoMovimento = 'x';
    } else if (c == '?' || c == 'h') {
      imprimirAjuda();
    } else if (strchr("wsadqex", c)) {
      executarMovimento(c);
      Serial.printf("Movimento: %c\n", c);
    }
  }
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);

  for (auto m : motores) m->iniciar();

  imprimirAjuda();

  if (LARGADA_AUTOMATICA) {
    Serial.printf("Largada automatica em %lu ms...\n", (unsigned long)ATRASO_LARGADA_MS);
    uint32_t inicio = millis();
    while (millis() - inicio < ATRASO_LARGADA_MS) {
      lerSerial();
      if (comandoRecebido) {
        Serial.println(F("Largada automatica cancelada (comando recebido)."));
        return;
      }
      delay(10);
    }
    executarMovimento('w');
    Serial.println(F("VAI! Andando para frente."));
  }
}

void loop() {
  lerSerial();
  atualizarMotores();

  // --------------------------------------------------------------------------
  //  Aqui entra a lógica da pista (sensores de obstáculo, linha etc.).
  //  Exemplo com sensor de obstáculo no D13:
  //
  //    if (digitalRead(13) == LOW) {   // obstáculo detectado
  //      curvaDireita(velocidadeAtual);
  //    } else {
  //      frente(velocidadeAtual);
  //    }
  // --------------------------------------------------------------------------
}
