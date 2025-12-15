#include <Servo.h>

// ==========================
// Pines de los servos
// ==========================
const int PIN_Q1      = 7;
const int PIN_Q2      = 8;
const int PIN_Q3      = 9;
const int PIN_GRIPPER = 10;  

Servo servoQ1;
Servo servoQ2;
Servo servoQ3;
Servo servoGripper;

// ==========================
// Calibraciones y límites
// ==========================

// Cero físico medido: ~83° en q1 y q2
const int SERVO_Q1_CENTER = 82;   // q1 = 0°
const int SERVO_Q2_CENTER = 87;   // q2 = 0° cuando q1=0 y q2=0

// Signos (por el montaje físico)
const int SIGN_Q1 = +1;   // base normal
const int SIGN_Q2 = -1;   // servo de q2 volteado de cabeza → invertido

// Límites de servos (para no forzarlos)
const int SERVO_Q1_MIN = 30;
const int SERVO_Q1_MAX = 150;

const int SERVO_Q2_MIN = 0;
const int SERVO_Q2_MAX = 180;

// Q2 solo puede aportar un extra de ±30°
const float Q2_EXTRA_MAX_DEG = 60.0;

// Mapeo de q3 (m) al servo (prismático)
const float Q3_MIN_M = 0.0;    // mínimo del modelo (m)
const float Q3_MAX_M = 0.06;   // máximo del modelo (m)

const int SERVO_Q3_MIN = 0;    // grados servo cuando q3 = Q3_MIN_M
const int SERVO_Q3_MAX = 170;  // grados servo cuando q3 = Q3_MAX_M

// ==========================
// Gripper (servo)
// ==========================
// AJUSTA ESTOS ÁNGULOS SEGÚN TU MECANISMO
const int GRIPPER_OPEN_ANGLE  = 0;  // gripper abierto
const int GRIPPER_CLOSE_ANGLE = 50;  // gripper cerrado

// Estado actual de los servos (para movimiento suave)
int current_servo_q1;
int current_servo_q2;
int current_servo_q3;

// Prototipo
void moveServosSmooth(int target_q1, int target_q2, int target_q3, float speed_factor);

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(115200);

  servoQ1.attach(PIN_Q1);
  servoQ2.attach(PIN_Q2);
  servoQ3.attach(PIN_Q3);
  servoGripper.attach(PIN_GRIPPER);

  // Posición inicial neutra (centro en q1/q2, mínimo en q3, gripper abierto)
  servoQ1.write(SERVO_Q1_CENTER);
  servoQ2.write(SERVO_Q2_CENTER);
  servoQ3.write(SERVO_Q3_MIN);
  servoGripper.write(GRIPPER_OPEN_ANGLE);

  current_servo_q1 = SERVO_Q1_CENTER;
  current_servo_q2 = SERVO_Q2_CENTER;
  current_servo_q3 = SERVO_Q3_MIN;

  Serial.println("SCARA listo. Esperando: q1_rad,q2_rad,q3_m,g,vel");
}

// ==========================
// Loop principal
// ==========================
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // Debug crudo
    Serial.print("RX line: ");
    Serial.println(line);

    // Formatos aceptados:
    // - q1,q2,q3
    // - q1,q2,q3,g
    // - q1,q2,q3,g,vel
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1 + 1);
    if (i1 == -1 || i2 == -1) {
      Serial.println("Formato invalido");
      return;
    }
    int i3 = line.indexOf(',', i2 + 1);
    int i4 = (i3 != -1) ? line.indexOf(',', i3 + 1) : -1;

    String s_q1 = line.substring(0, i1);
    String s_q2 = line.substring(i1 + 1, i2);
    String s_q3;
    String s_g;
    String s_speed;

    if (i3 == -1) {
      // Formato antiguo: q1,q2,q3
      s_q3 = line.substring(i2 + 1);
    } else {
      // Nuevo: q1,q2,q3,g[ ,vel]
      s_q3 = line.substring(i2 + 1, i3);
      if (i4 == -1) {
        // solo g
        s_g = line.substring(i3 + 1);
      } else {
        // g y vel
        s_g = line.substring(i3 + 1, i4);
        s_speed = line.substring(i4 + 1);
      }
    }

    float q1_rad = s_q1.toFloat();
    float q2_rad = s_q2.toFloat();
    float q3_m   = s_q3.toFloat();
    int   g      = s_g.length() ? s_g.toInt() : 1;      // por defecto gripper abierto
    float speed_factor = s_speed.length() ? s_speed.toFloat() : 1.0;

    // ===============================
    // 1. Convertir q1, q2 a grados
    // ===============================
    float q1_deg_model = q1_rad * 180.0 / PI;  // modelo
    float q2_deg_model = q2_rad * 180.0 / PI;  // "extra" de codo

    // Limitamos el "extra" de q2 a ±30°
    float q2_extra_deg = q2_deg_model;
    if (q2_extra_deg >  Q2_EXTRA_MAX_DEG) q2_extra_deg =  Q2_EXTRA_MAX_DEG;
    if (q2_extra_deg < -Q2_EXTRA_MAX_DEG) q2_extra_deg = -Q2_EXTRA_MAX_DEG;

    // ===============================
    // 2. Servo de q1
    // ===============================
    float servo_q1_f = SERVO_Q1_CENTER + SIGN_Q1 * q1_deg_model;
    int   servo_q1   = (int)servo_q1_f;
    servo_q1 = constrain(servo_q1, SERVO_Q1_MIN, SERVO_Q1_MAX);

    // ===============================
    // 3. Servo de q2 (codo)
    // ===============================
    float theta2_virtual = q1_deg_model + q2_extra_deg;  // "q1 + extra"
    float servo_q2_f = SERVO_Q2_CENTER + SIGN_Q2 * theta2_virtual;
    int   servo_q2   = (int)servo_q2_f;
    servo_q2 = constrain(servo_q2, SERVO_Q2_MIN, SERVO_Q2_MAX);

    // ===============================
    // 4. Convertir q3 (m) a grados servo (prismático)
    // ===============================
    float ratio = (q3_m - Q3_MIN_M) / (Q3_MAX_M - Q3_MIN_M);
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    int servo_q3 = (int)(SERVO_Q3_MIN + ratio * (SERVO_Q3_MAX - SERVO_Q3_MIN));

    // ===============================
    // 5. Gripper (servo)
// ===============================
    if (g == 1) {
      servoGripper.write(GRIPPER_OPEN_ANGLE);
    } else {
      servoGripper.write(GRIPPER_CLOSE_ANGLE);
    }

    // ===============================
    // 6. Mover servos con velocidad
    // ===============================
    moveServosSmooth(servo_q1, servo_q2, servo_q3, speed_factor);

    // ===============================
    // 7. Debug por Serial
    // ===============================
    Serial.print("q1_rad="); Serial.print(q1_rad, 4);
    Serial.print("  q2_rad="); Serial.print(q2_rad, 4);
    Serial.print("  q3_m=");   Serial.print(q3_m, 4);
    Serial.print("  g=");      Serial.print(g);
    Serial.print("  vel=");    Serial.print(speed_factor, 2);

    Serial.print("  -> servoQ1="); Serial.print(servo_q1);
    Serial.print("  servoQ2=");    Serial.print(servo_q2);
    Serial.print("  servoQ3=");    Serial.println(servo_q3);
  }
}

// ==========================
// Movimiento suave con velocidad
// ==========================
void moveServosSmooth(int target_q1, int target_q2, int target_q3, float speed_factor) {
  if (speed_factor <= 0.0) speed_factor = 0.1;
  if (speed_factor > 3.0)  speed_factor = 3.0;

  int delta1 = abs(target_q1 - current_servo_q1);
  int delta2 = abs(target_q2 - current_servo_q2);
  int delta3 = abs(target_q3 - current_servo_q3);

  int max_delta = delta1;
  if (delta2 > max_delta) max_delta = delta2;
  if (delta3 > max_delta) max_delta = delta3;

  if (max_delta == 0) return;

  int   steps        = max_delta;     // 1° por paso
  float base_delayMs = 10.0;          // a vel=1.0 → 10 ms por paso
  float delayMs      = base_delayMs / speed_factor;

  for (int i = 1; i <= steps; i++) {
    int q1 = current_servo_q1 + (target_q1 - current_servo_q1) * i / steps;
    int q2 = current_servo_q2 + (target_q2 - current_servo_q2) * i / steps;
    int q3 = current_servo_q3 + (target_q3 - current_servo_q3) * i / steps;

    servoQ1.write(q1);
    servoQ2.write(q2);
    servoQ3.write(q3);

    delay((unsigned long)delayMs);
  }

  current_servo_q1 = target_q1;
  current_servo_q2 = target_q2;
  current_servo_q3 = target_q3;
}
