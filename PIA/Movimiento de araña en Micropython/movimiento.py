"""
Robot cuadrúpedo con interpolación suave - MicroPython
Driver: PCA9685 vía I2C (dirección 0x40)
Plataforma: ESP32
"""

import math
import time
import sys
import select
from machine import I2C, Pin


# ── Driver PCA9685 ─────────────────────────────────────────────

class PCA9685:
    MODE1     = 0x00
    PRESCALE  = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, i2c, address=0x40):
        self.i2c  = i2c
        self.addr = address
        self._write(self.MODE1, 0x00)
        time.sleep_ms(10)
        self.set_pwm_freq(50)

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def set_pwm_freq(self, freq_hz):
        prescale = int(math.floor(25_000_000.0 / (4096.0 * freq_hz) - 0.5))
        old = self._read(self.MODE1)
        self._write(self.MODE1, (old & 0x7F) | 0x10)
        self._write(self.PRESCALE, prescale)
        self._write(self.MODE1, old)
        time.sleep_ms(5)
        self._write(self.MODE1, old | 0xA0)

    def set_pwm(self, channel, on, off):
        reg  = self.LED0_ON_L + 4 * channel
        data = bytes([on & 0xFF, (on >> 8) & 0xFF, off & 0xFF, (off >> 8) & 0xFF])
        self.i2c.writeto_mem(self.addr, reg, data)


# ── Configuración ──────────────────────────────────────────────

SERVOMIN   = 150
SERVOMAX   = 600
NUM_SERVOS = 12

KEEP = 255.0    # centinela: "no cambiar este eje"

# Dimensiones del robot (mm)
LENGTH_A = 55.0
LENGTH_B = 77.5
LENGTH_C = 27.5

X_DEFAULT = 62.0
X_OFFSET  = 0.0
Y_START   = 0.0
Y_STEP    = 40.0
Z_DEFAULT = -50.0
Z_UP      = -30.0
Z_BOOT    = -28.0

# Velocidades (unidades/tick, 1 tick = 20 ms)
SPOT_TURN_SPEED  = 4.0
LEG_MOVE_SPEED   = 5.0
BODY_MOVE_SPEED  = 5.0
STAND_SEAT_SPEED = 1.0

# ── Estado global ──────────────────────────────────────────────

# site_now[pata][eje]  — posición actual interpolada
site_now    = [[0.0, 0.0, 0.0] for _ in range(4)]
# site_expect[pata][eje] — posición destino
site_expect = [[0.0, 0.0, 0.0] for _ in range(4)]
# temp_speed[pata][eje]  — velocidad por eje para llegar al destino
temp_speed  = [[0.0, 0.0, 0.0] for _ in range(4)]

move_speed = LEG_MOVE_SPEED
angulos    = [90] * NUM_SERVOS

# ── Hardware ───────────────────────────────────────────────────

i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400_000)
pca = PCA9685(i2c, address=0x40)


# ── Utilidades de servo ────────────────────────────────────────

def angle_to_pulse(angle):
    return int(SERVOMIN + (SERVOMAX - SERVOMIN) * angle / 180)

def mover_servo(canal, angulo):
    angulo = max(0, min(180, int(round(angulo))))
    angulos[canal] = angulo
    pca.set_pwm(canal, 0, angle_to_pulse(angulo))

def mostrar_estado():
    print("=== Estado de servos ===")
    for i in range(NUM_SERVOS):
        print(f"  Servo {i:>2}: {angulos[i]:>3}°")
    print("========================")

def reset_servos():
    for i in range(NUM_SERVOS):
        mover_servo(i, 90)
        time.sleep_ms(50)
    print("Todos los servos en 90°")


# ── Cinemática inversa ─────────────────────────────────────────

def cartesian_to_polar(x, y, z):
    w  = (1.0 if x >= 0 else -1.0) * math.sqrt(x*x + y*y)
    v  = w - LENGTH_C
    vz = math.sqrt(v*v + z*z)

    alpha = (math.atan2(z, v)
             + math.acos(
                 (LENGTH_A**2 - LENGTH_B**2 + v**2 + z**2)
                 / (2.0 * LENGTH_A * vz)
             ))
    beta  = math.acos(
                (LENGTH_A**2 + LENGTH_B**2 - v**2 - z**2)
                / (2.0 * LENGTH_A * LENGTH_B)
            )
    gamma = math.atan2(y, x) if w >= 0 else math.atan2(-y, -x)

    return math.degrees(alpha), math.degrees(beta), math.degrees(gamma)

def polar_to_servo(leg, alpha, beta, gamma):
    if leg == 0:
        a, b, g = 90 - alpha, beta,       gamma + 90
    elif leg == 1:
        a, b, g = 90 + alpha, 180 - beta, 90 - gamma
    elif leg == 2:
        a, b, g = 90 + alpha, 180 - beta, 90 - gamma
    else:
        a, b, g = 90 - alpha, beta,       gamma + 90

    base = leg * 3
    mover_servo(base,     a)
    mover_servo(base + 1, b)
    mover_servo(base + 2, g)


# ── Motor de interpolación ─────────────────────────────────────

def servo_service():
    """Un tick de interpolación: avanza cada pata un paso hacia su destino."""
    for i in range(4):
        for j in range(3):
            diff = site_expect[i][j] - site_now[i][j]
            if abs(diff) >= abs(temp_speed[i][j]):
                site_now[i][j] += temp_speed[i][j]
            else:
                site_now[i][j] = site_expect[i][j]

        alpha, beta, gamma = cartesian_to_polar(
            site_now[i][0], site_now[i][1], site_now[i][2]
        )
        polar_to_servo(i, alpha, beta, gamma)

def wait_all_reach():
    """Llama servo_service cada 20 ms hasta que todas las patas lleguen al destino."""
    while True:
        servo_service()
        time.sleep_ms(20)
        reached = all(
            site_now[i][j] == site_expect[i][j]
            for i in range(4)
            for j in range(3)
        )
        if reached:
            break

def set_site(leg, x, y, z):
    """
    Define el destino de una pata y calcula la velocidad por eje.
    Usar KEEP en cualquier coordenada para no modificarla.
    """
    global move_speed

    lx = (x - site_now[leg][0]) if x != KEEP else 0.0
    ly = (y - site_now[leg][1]) if y != KEEP else 0.0
    lz = (z - site_now[leg][2]) if z != KEEP else 0.0

    length = math.sqrt(lx*lx + ly*ly + lz*lz)

    if length == 0:
        temp_speed[leg][0] = 0.0
        temp_speed[leg][1] = 0.0
        temp_speed[leg][2] = 0.0
    else:
        temp_speed[leg][0] = lx / length * move_speed
        temp_speed[leg][1] = ly / length * move_speed
        temp_speed[leg][2] = lz / length * move_speed

    if x != KEEP: site_expect[leg][0] = x
    if y != KEEP: site_expect[leg][1] = y
    if z != KEEP: site_expect[leg][2] = z


# ── Movimientos ────────────────────────────────────────────────

def stand():
    global move_speed
    move_speed = STAND_SEAT_SPEED
    for leg in range(4):
        set_site(leg, KEEP, KEEP, Z_DEFAULT)
    wait_all_reach()

def sit():
    global move_speed
    move_speed = STAND_SEAT_SPEED
    for leg in range(4):
        set_site(leg, KEEP, KEEP, Z_BOOT)
    wait_all_reach()

def step_forward(pasos=1):
    global move_speed

    for _ in range(pasos):
        if site_now[2][1] == Y_START:
            # ── Pata 2 avanza ─────────────────────────────────
            move_speed = LEG_MOVE_SPEED
            set_site(2, X_DEFAULT, Y_START,              Z_UP);      wait_all_reach()
            set_site(2, X_DEFAULT, Y_START + 2*Y_STEP,  Z_UP);      wait_all_reach()
            set_site(2, X_DEFAULT, Y_START + 2*Y_STEP,  Z_DEFAULT); wait_all_reach()

            move_speed = BODY_MOVE_SPEED
            set_site(0, X_DEFAULT, Y_START,              Z_DEFAULT)
            set_site(1, X_DEFAULT, Y_START + 2*Y_STEP,  Z_DEFAULT)
            set_site(2, X_DEFAULT, Y_START + Y_STEP,    Z_DEFAULT)
            set_site(3, X_DEFAULT, Y_START + Y_STEP,    Z_DEFAULT)
            wait_all_reach()

            move_speed = LEG_MOVE_SPEED
            set_site(1, X_DEFAULT, Y_START + 2*Y_STEP,  Z_UP);      wait_all_reach()
            set_site(1, X_DEFAULT, Y_START,              Z_UP);      wait_all_reach()
            set_site(1, X_DEFAULT, Y_START,              Z_DEFAULT); wait_all_reach()

        else:
            # ── Pata 0 avanza ─────────────────────────────────
            move_speed = LEG_MOVE_SPEED
            set_site(0, X_DEFAULT, Y_START,              Z_UP);      wait_all_reach()
            set_site(0, X_DEFAULT, Y_START + 2*Y_STEP,  Z_UP);      wait_all_reach()
            set_site(0, X_DEFAULT, Y_START + 2*Y_STEP,  Z_DEFAULT); wait_all_reach()

            move_speed = BODY_MOVE_SPEED
            set_site(0, X_DEFAULT, Y_START + Y_STEP,    Z_DEFAULT)
            set_site(1, X_DEFAULT, Y_START + Y_STEP,    Z_DEFAULT)
            set_site(2, X_DEFAULT, Y_START,              Z_DEFAULT)
            set_site(3, X_DEFAULT, Y_START + 2*Y_STEP,  Z_DEFAULT)
            wait_all_reach()

            move_speed = LEG_MOVE_SPEED
            set_site(3, X_DEFAULT, Y_START + 2*Y_STEP,  Z_UP);      wait_all_reach()
            set_site(3, X_DEFAULT, Y_START,              Z_UP);      wait_all_reach()
            set_site(3, X_DEFAULT, Y_START,              Z_DEFAULT); wait_all_reach()

def body_dance(ciclos=8):
    global move_speed

    y_mid            = X_DEFAULT
    body_dance_speed = 2.0

    sit()

    move_speed = 1.0
    for leg in range(4):
        set_site(leg, X_DEFAULT, y_mid, KEEP)
    wait_all_reach()

    for leg in range(4):
        set_site(leg, X_DEFAULT, y_mid, Z_DEFAULT - 20)
    wait_all_reach()

    # head_up
    move_speed = body_dance_speed
    set_site(0, KEEP, KEEP, site_now[0][2] - 30)
    set_site(1, KEEP, KEEP, site_now[1][2] + 30)
    set_site(2, KEEP, KEEP, site_now[2][2] - 30)
    set_site(3, KEEP, KEEP, site_now[3][2] + 30)
    wait_all_reach()

    for j in range(ciclos):
        if   j > ciclos // 2: move_speed = body_dance_speed * 3
        elif j > ciclos // 4: move_speed = body_dance_speed * 2
        else:                 move_speed = body_dance_speed

        set_site(0, KEEP, y_mid - 20, KEEP)
        set_site(1, KEEP, y_mid + 20, KEEP)
        set_site(2, KEEP, y_mid - 20, KEEP)
        set_site(3, KEEP, y_mid + 20, KEEP)
        wait_all_reach()

        set_site(0, KEEP, y_mid + 20, KEEP)
        set_site(1, KEEP, y_mid - 20, KEEP)
        set_site(2, KEEP, y_mid + 20, KEEP)
        set_site(3, KEEP, y_mid - 20, KEEP)
        wait_all_reach()

    # head_down
    move_speed = body_dance_speed
    set_site(0, KEEP, KEEP, site_now[0][2] + 30)
    set_site(1, KEEP, KEEP, site_now[1][2] - 30)
    set_site(2, KEEP, KEEP, site_now[2][2] + 30)
    set_site(3, KEEP, KEEP, site_now[3][2] - 30)
    wait_all_reach()


# ── Lectura de comandos ────────────────────────────────────────

def procesar_comando(cmd):
    cmd = cmd.strip()
    if cmd == "":
        return

    if cmd == "v":
        mostrar_estado()

    elif cmd == "reset":
        reset_servos()

    elif cmd == "stand":
        stand()
        print("Stand ejecutado")
        mostrar_estado()

    elif cmd.startswith("forward"):
        try:
            pasos = int(cmd[7:].strip()) if len(cmd) > 7 else 1
        except ValueError:
            pasos = 1
        pasos = max(1, pasos)
        print(f"Avanzando {pasos} pasos...")
        step_forward(pasos)
        print("Listo")

    elif cmd.startswith("dance"):
        try:
            ciclos = int(cmd[5:].strip()) if len(cmd) > 5 else 8
        except ValueError:
            ciclos = 8
        ciclos = max(1, ciclos)
        print(f"Bailando {ciclos} ciclos...")
        body_dance(ciclos)
        print("Listo")

    elif cmd.startswith("s "):
        partes = cmd.split()
        if len(partes) == 3:
            try:
                canal  = int(partes[1])
                angulo = int(partes[2])
                if 0 <= canal < NUM_SERVOS:
                    mover_servo(canal, angulo)
                    print(f"Servo {canal} → {angulos[canal]}°")
                else:
                    print(f"Canal inválido (0-{NUM_SERVOS - 1})")
            except ValueError:
                print("Uso: s <canal> <angulo>")
        else:
            print("Uso: s <canal> <angulo>")

    else:
        print("Comandos disponibles:")
        print("  stand              → posicion de parado")
        print("  forward <pasos>    → avanzar N pasos")
        print("  dance <ciclos>     → baile (default 8)")
        print("  reset              → todos los servos a 90°")
        print("  s <canal> <angulo> → mover servo directo")
        print("  v                  → ver estado de servos")


# ── Main ───────────────────────────────────────────────────────

def main():
    global move_speed

    # Inicializar posiciones igual que el setup original
    _init_positions = [
        (0, X_DEFAULT, Y_START + Y_STEP, Z_BOOT),
        (1, X_DEFAULT, Y_START + Y_STEP, Z_BOOT),
        (2, X_DEFAULT, Y_START,          Z_BOOT),
        (3, X_DEFAULT, Y_START,          Z_BOOT),
    ]
    for leg, x, y, z in _init_positions:
        site_now[leg]    = [x, y, z]
        site_expect[leg] = [x, y, z]

    # Escribir posición inicial a los servos
    for i in range(4):
        alpha, beta, gamma = cartesian_to_polar(
            site_now[i][0], site_now[i][1], site_now[i][2]
        )
        polar_to_servo(i, alpha, beta, gamma)

    stand()

    print("Listo. Comandos:")
    print("  stand              → posicion de parado")
    print("  forward <pasos>    → avanzar N pasos")
    print("  dance <ciclos>     → baile (default 8)")
    print("  reset              → todos los servos a 90°")
    print("  s <canal> <angulo> → mover servo directo")
    print("  v                  → ver estado de servos")

    buf = ""
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            char = sys.stdin.read(1)
            if char in ('\n', '\r'):
                procesar_comando(buf)
                buf = ""
            else:
                buf += char
        time.sleep_ms(10)


main()