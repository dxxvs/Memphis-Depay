# Robô Car Race – ESP32 + 2x L298N + 4 motores DC

Código em `robo_car_race/robo_car_race.ino` (Arduino IDE, placa **ESP32 Dev Module**).

## Ligações

| L298N FRENTE | ESP32 | | L298N TRASEIRA | ESP32 |
|---|---|---|---|---|
| ENA (PWM diant. esq.) | D32 | | ENA (PWM tras. esq.) | D4 |
| IN1 | D33 | | IN1 | RX2 (GPIO16) |
| IN2 | D25 | | IN2 | TX2 (GPIO17) |
| IN3 | D26 | | IN3 | D18 |
| IN4 | D27 | | IN4 | D19 |
| ENB (PWM diant. dir.) | D14 | | ENB (PWM tras. dir.) | D21 |

- **Tire os jumpers ENA/ENB** das duas L298N (senão o PWM não funciona).
- Bateria 9 V (+) → chave → borne **12V** das duas L298N; (−) → **GND** das duas.
- **GND da L298N → GND do ESP32** (terra comum é obrigatório).
- **5V** da L298N → **VIN** do ESP32 (jumper do regulador 5V da L298N colocado).

### Sensores ultrassônicos HC-SR04 (frente + 2 diagonais a 20°)

| Sensor | TRIG | ECHO (com divisor!) |
|---|---|---|
| Frente | D13 | D34 |
| Esquerda (20°) | D22 | D35 |
| Direita (20°) | D23 | VP (GPIO36) |

- VCC dos sensores → 5V; GND → GND comum.
- **O ECHO sai com 5 V e o ESP32 só aguenta 3,3 V.** Em cada ECHO, ligue um
  divisor: `ECHO → 1 kΩ → pino do ESP32` e `pino do ESP32 → 2 kΩ → GND`.

## Como o piloto automático decide

- **Diagonais:** compara esquerda × direita e vira para o lado com mais espaço.
  Isso faz o carro seguir as curvas e ficar no meio da pista.
- **Frente:** livre = velocidade máxima; perto = desacelera e reforça a curva.
- **Diagonal muito perto** (< 25 cm): desvio forte para o outro lado.
- **Frente bloqueada** (< 15 cm): ré por 0,3 s e gira para o lado mais livre.
  Se bloquear de novo logo em seguida, tenta o outro lado.
- **Sensor sem resposta:** o carro para e avisa no Monitor Serial qual é.

Todas as distâncias e ganhos ficam no topo do código (seção PILOTO AUTOMÁTICO).

## Como usar

1. Arduino IDE → instale "esp32 by Espressif" (versão 2.x ou 3.x funcionam).
2. Placa: *ESP32 Dev Module*. Grave o código.
3. Ao ligar, espera 3 s e liga o piloto automático com `VELOCIDADE_CORRIDA` (0–255).
4. Monitor Serial 115200: `r` correr (piloto automático), `u` telemetria dos
   sensores, `w` frente, `s` ré, `a`/`d` girar, `q`/`e` curva, `x` parar,
   espaço = parar já, `0`–`9` velocidade, `t` testa cada motor.
   Qualquer comando manual desliga o piloto automático.

Ajustes no topo do código: velocidade, rampa, PWM mínimo, inverter motor e
`TRIM_*` (para o carro andar reto).

## Atenção com a bateria de 9 V

Bateria de 9 V "quadradinha" fornece pouca corrente; 4 motores TT puxam
0,8–1,5 A juntos (mais na partida). A L298N ainda perde ~2 V. Sintomas: carro
fraco e ESP32 reiniciando. A rampa de aceleração do código ajuda, mas o ideal
é usar 2x 18650 (7,4 V) ou um pack de 6x AA.
