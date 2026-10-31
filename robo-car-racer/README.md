# Robo Car Racer – ESP32

Versão de teste **sem servos**: rodas retas, o carrinho anda reto e faz as curvas parando ou invertendo os motores de um lado (igual a um tanque). Quando acha obstáculo, gira para o lado livre; se estiver muito perto, para, dá ré, gira e segue.

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
| IN1 (motor diant. esquerdo) | D22 |
| IN2 (motor diant. esquerdo) | D21 |
| IN3 (motor diant. direito) | D19 |
| IN4 (motor diant. direito) | D18 |

### Ponte H 2 – TRÁS (motores traseiros)
| L298N | ESP32 |
|------|-------|
| IN1 (motor tras. esquerdo) | RX2 (16) |
| IN2 (motor tras. esquerdo) | TX2 (17) |
| IN3 (motor tras. direito) | D15 |
| IN4 (motor tras. direito) | D2 |

> **ENA e ENB não são usados: deixe os jumpers neles** (vêm de fábrica). Os motores andam sempre na velocidade máxima.
>
> **GND do ESP32, das duas pontes H e da bateria têm que estar todos ligados juntos.**

### Sensores HC-SR04
| Sensor | TRIG | ECHO |
|--------|------|------|
| Esquerdo | D25 | VN (39) |
| Centro | D33 | D34 |
| Direito | D32 | D35 |

VCC dos sensores no **5V** (VIN). O ECHO sai em 5V: o ideal é colocar um divisor de tensão (resistor de 1k em série + 2k para o GND) em cada ECHO.

## Como usar

1. Liga o carrinho com as rodas para frente.
2. Abra o **Monitor Serial (115200)** – ele mostra as distâncias `E`, `C`, `D`. Passe a mão na frente de cada sensor para ver se estão certos.
3. Aperte o botão **BOOT** da placa → depois de 2 s ele larga.

## Calibração

- **Motor girando ao contrário?** Mude `INV_DIANT_ESQ`, `INV_DIANT_DIR`, `INV_TRAS_ESQ` ou `INV_TRAS_DIR` para `true`.
- **Desvia tarde demais?** Aumente `DIST_FRENTE_VIRAR` e `DIST_FRENTE_RE`.
- **Gira pouco/muito depois da ré?** Ajuste `TEMPO_GIRO_MS`.
- **Ré curta/longa?** Ajuste `TEMPO_RE_MS`.
- **Raspando na parede?** Aumente `DIST_LATERAL_MIN`.
- Para a competição, mude `DEBUG` para `false`.
