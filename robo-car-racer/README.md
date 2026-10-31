# Robo Car Racer – ESP32

Código para o carrinho andar reto, ficar no meio da pista e desviar de obstáculos usando os 3 sensores ultrassônicos.

Arquivo: `robo_car_racer/robo_car_racer.ino` (não precisa de nenhuma biblioteca extra).

## Como gravar

1. Arduino IDE → **Ferramentas › Placa › Gerenciador de placas** → instale **esp32 (Espressif)**.
2. Placa: **ESP32 Dev Module**. Porta: a COM/USB do carrinho.
3. Abra `robo_car_racer.ino` e clique em **Carregar**.
   (Se ficar em "Connecting...", segure o botão **BOOT** até começar a gravar.)

## Ligações

Se a sua ligação for diferente, mude só os números no topo do código (seção `CONFIGURACOES - PINOS`).

### Ponte H 1 – FRENTE (motores dianteiros)
| L298N | ESP32 |
|------|-------|
| ENA (motor diant. esquerdo) | D23 |
| IN1 | D22 |
| IN2 | D21 |
| IN3 | D19 |
| IN4 | D18 |
| ENB (motor diant. direito) | D5 |

### Ponte H 2 – TRÁS (motores traseiros)
| L298N | ESP32 |
|------|-------|
| ENA (motor tras. esquerdo) | D4 |
| IN1 | RX2 (16) |
| IN2 | TX2 (17) |
| IN3 | D15 |
| IN4 | D2 |
| ENB (motor tras. direito) | D26 |

> **Tire os jumpers do ENA e ENB** das pontes H e ligue esses pinos no ESP32. Com o jumper o motor fica sempre na velocidade máxima e o carrinho não consegue fazer curva suave.
>
> **GND do ESP32, das duas pontes H e da bateria têm que estar todos ligados juntos.**

### Sensores HC-SR04
| Sensor | TRIG | ECHO |
|--------|------|------|
| Esquerdo | D25 | VN (39) |
| Centro | D33 | D34 |
| Direito | D32 | D35 |

VCC dos sensores no **5V** (VIN). O ECHO sai em 5V: o ideal é colocar um divisor de tensão (resistor de 1k em série + 2k para o GND) em cada ECHO.

### Servos
| Servo | Sinal |
|-------|-------|
| Servo 1 (roda dianteira esquerda) | D13 |
| Servo 2 (roda dianteira direita) | D14 |
| Servo 3 | D27 |

Alimente os servos pelo 5V do regulador/bateria (não pelo 3V3 do ESP32).

## Como usar

1. Liga o carrinho com as rodas para frente.
2. Abra o **Monitor Serial (115200)** – ele mostra as distâncias `E`, `C`, `D`. Passe a mão na frente de cada sensor para ver se estão certos.
3. Aperte o botão **BOOT** da placa → depois de 2 s ele larga.

## Calibração (faça nesta ordem)

1. **Motores girando ao contrário?** Mude `INV_DIANT_ESQ`, `INV_DIANT_DIR`, `INV_TRAS_ESQ`, `INV_TRAS_DIR` para `true` no motor que estiver errado.
2. **Rodas não ficam retas?** Ajuste `SERVO1_CENTRO` / `SERVO2_CENTRO` (ex.: 85, 95...).
3. **Roda vira para o lado errado?** Troque `SERVO1_SINAL` ou `SERVO2_SINAL` para `-1`.
4. **Terceiro servo:** `SERVO3_MODO = 0` (fica parado), `1` (vira junto com as da frente) ou `2` (vira ao contrário – direção nas 4 rodas, curva mais fechada).
5. **Velocidade:** `VEL_RETA` e `VEL_CURVA` (0–255). Se o motor só zumbe sem girar, aumente `PWM_MINIMO`.
6. **Comportamento:**
   - Ficando em zigue-zague → diminua `KP_CENTRALIZAR` ou `SUAVIZACAO`, ou aumente `ZONA_MORTA`.
   - Demora para desviar → aumente `DIST_FRENTE_ALERTA` ou `GANHO_FRENTE`.
   - Raspando na parede → aumente `DIST_LATERAL_MIN` ou `GANHO_PAREDE`.
   - Curva muito aberta → aumente `GANHO_DIFERENCIAL` e/ou `SERVO_MAX_GRAUS`.
7. Para a competição, mude `DEBUG` para `false`.
