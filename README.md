# Robot-SCARA-3GDL-RRP
# Robot SCARA RRP (3 GDL) – GUI en Python + Control con Arduino (Serial)

Repositorio del proyecto para controlar y visualizar un **robot SCARA RRP de 3 grados de libertad** (R–R–P) usando **una GUI en Python** y un **Arduino** como controlador de servomotores. Incluye cálculo de cinemática (FK y soporte de IK), simulación/visualización y envío de comandos por **puerto serial** al robot físico.

---

## 1) Descripción del robot

El robot implementado es un **SCARA RRP (3 GDL)**:
- **q1 (Revoluta)**: giro de la base (plano XY)
- **q2 (Revoluta)**: giro del “codo” (plano XY)
- **d3 (Prismática)**: movimiento vertical (eje Z) *implementado con un servo de 180° acoplado a un mecanismo lineal*

**Actuadores usados (prototipo real):**
- **q1 y q2:** Servomotor **MG996R**
- **d3 (prismático) y pinza/gripper:** Servomotor **SG90**

**Aplicación demostrativa:** manipulación básica de piezas (tomar y colocar) a diferentes posiciones y **alturas** dentro del espacio de trabajo del robot.

---

## 2) Contenido del repositorio

Este repositorio contiene:
- **Código Arduino**: recepción por serial y control de servos
- **Código Python (GUI)**: control manual, cálculo cinemático, visualización y envío serial
- **Evidencia**: GIF/Video y fotos del prototipo (opcional pero recomendado)

---

## 3) Archivos principales
-arduino_scara.ino # Arduino: receptor serial + control servos
-gui_scara.py # Python GUI 

---

## 4) Requisitos

### Hardware
- Arduino Uno (o equivalente)
- 2× **MG996R** (q1, q2)
- 2× **SG90** (d3, pinza)
- Fuente externa regulada **5 V** con corriente suficiente (recomendado para MG996R)
- Protoboard/cables
- Robot SCARA físico (estructura + eje Z + pinza)

### Software
- Arduino IDE
- Python 3.9+ (recomendado)
- Librerías Python:
  - `numpy`
  - `matplotlib`
  - `pyserial`

> Puedes renombrar, pero se recomienda mantener la estructura para evitar errores de importación.

