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

## Como usar

1. Arduino IDE → instale "esp32 by Espressif" (versão 2.x ou 3.x funcionam).
2. Placa: *ESP32 Dev Module*. Grave o código.
3. Ao ligar, espera 3 s e sai para frente com `VELOCIDADE_CORRIDA` (0–255).
4. Monitor Serial 115200: `w` frente, `s` ré, `a`/`d` girar, `q`/`e` curva,
   `x` parar, espaço = parar já, `0`–`9` velocidade, `t` testa cada motor.

Ajustes no topo do código: velocidade, rampa, PWM mínimo, inverter motor e
`TRIM_*` (para o carro andar reto).

## Atenção com a bateria de 9 V

Bateria de 9 V "quadradinha" fornece pouca corrente; 4 motores TT puxam
0,8–1,5 A juntos (mais na partida). A L298N ainda perde ~2 V. Sintomas: carro
fraco e ESP32 reiniciando. A rampa de aceleração do código ajuda, mas o ideal
é usar 2x 18650 (7,4 V) ou um pack de 6x AA.
