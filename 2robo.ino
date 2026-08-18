/*
  Mapeamento informado:
    Sensor 12 -> D30 -> FRENTE
    Sensor 11 -> D31 -> FRENTE_ESQUERDA
    Sensor 10 -> D32 -> FRENTE_ESQUERDA
    Sensor 09 -> D33 -> ESQUERDA
    Sensor 08 -> D34 -> TRAS_ESQUERDA
    Sensor 07 -> D35 -> TRAS_ESQUERDA
    Sensor 06 -> D36 -> TRAS
    Sensor 05 -> D37 -> TRAS_DIREITA
    Sensor 04 -> D38 -> TRAS_DIREITA
    Sensor 03 -> D39 -> DIREITA
    Sensor 02 -> D40 -> FRENTE_DIREITA
*/

// ===== PINOS DOS MOTORES =====
#define in_1_ph_1 25
#define in_2_ph_1 24
#define in_3_ph_1 23
#define in_4_ph_1 22

#define in_1_ph_2 47
#define in_2_ph_2 46
#define in_3_ph_2 49
#define in_4_ph_2 48

#define pwm_2 2
#define pwm_1 3
#define pwm_4 4
#define pwm_3 5

// ===== SENTIDOS =====
#define HORARIO 1
#define ANTI_HORARIO -1
#define PARADO 0

// ===== DIRECOES DO INFRAVERMELHO =====
enum DirecaoIr {
  IR_FRENTE,
  IR_FRENTE_DIREITA,
  IR_DIREITA,
  IR_TRAS_DIREITA,
  IR_TRAS,
  IR_TRAS_ESQUERDA,
  IR_ESQUERDA,
  IR_FRENTE_ESQUERDA,
  IR_NAO_DETECTADA
};

// ===== INFRAVERMELHO =====
constexpr uint8_t SENSOR_COUNT = 11;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40};
constexpr uint8_t SENSOR_NUMBERS[SENSOR_COUNT] = {12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2};

const DirecaoIr SENSOR_DIRECTIONS[SENSOR_COUNT] = {
  IR_FRENTE,
  IR_FRENTE_ESQUERDA,
  IR_FRENTE_ESQUERDA,
  IR_ESQUERDA,
  IR_TRAS_ESQUERDA,
  IR_TRAS_ESQUERDA,
  IR_TRAS,
  IR_TRAS_DIREITA,
  IR_TRAS_DIREITA,
  IR_DIREITA,
  IR_FRENTE_DIREITA
};

const char *const directionName[] = {
  "FRENTE",
  "FRENTE_DIREITA",
  "DIREITA",
  "TRAS_DIREITA",
  "TRAS",
  "TRAS_ESQUERDA",
  "ESQUERDA",
  "FRENTE_ESQUERDA",
  "NAO_DETECTADA"
};

constexpr uint16_t WINDOW_US = 25000;
constexpr uint16_t DETECT_MIN_LOW_US = 800;
constexpr uint16_t SWITCH_MARGIN_US = 700;
constexpr uint8_t SWITCH_CONFIRMATIONS = 2;

uint16_t readings[SENSOR_COUNT];
uint16_t filteredReadings[SENSOR_COUNT];
bool filterInitialized = false;
uint8_t lastSensorIndex = 0;
uint8_t pendingSensorIndex = 0;
uint8_t pendingCount = 0;

// ===== CONFIGURACOES =====
const bool DEBUG_SERIAL_ATIVO = false;

int velocidadeFrente = 210;
int velocidadeGiro = 130;
int velocidadeBusca = 150;
int correcaoCurvaFrente = 70;

DirecaoIr direcaoIrAtual = IR_NAO_DETECTADA;
uint8_t sensorIrAtual = 0;
unsigned long ultimoPrintSerial = 0;

void setup() {
  if (DEBUG_SERIAL_ATIVO) {
    Serial.begin(115200);
  }

  pinMode(in_1_ph_1, OUTPUT);
  pinMode(in_2_ph_1, OUTPUT);
  pinMode(in_3_ph_1, OUTPUT);
  pinMode(in_4_ph_1, OUTPUT);

  pinMode(in_1_ph_2, OUTPUT);
  pinMode(in_2_ph_2, OUTPUT);
  pinMode(in_3_ph_2, OUTPUT);
  pinMode(in_4_ph_2, OUTPUT);

  pinMode(pwm_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  pinMode(pwm_3, OUTPUT);
  pinMode(pwm_4, OUTPUT);

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  stop_robot();

  if (DEBUG_SERIAL_ATIVO) {
    Serial.println("Robo iniciado apenas com movimentacao e infravermelho circular");
  }
}

void loop() {
  ir_reader();
  monitor_serial();
  seguir_bola_simples();

  delay(10);
}

void ir_reader() {
  readSensors();
  updateFilter();

  if (ballDetected()) {
    uint8_t index = getStableSensorIndex();
    sensorIrAtual = SENSOR_NUMBERS[index];
    direcaoIrAtual = SENSOR_DIRECTIONS[index];
  } else {
    sensorIrAtual = 0;
    direcaoIrAtual = IR_NAO_DETECTADA;
  }
}

void readSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    readings[i] = 0;
  }

  const uint32_t start = micros();
  uint32_t previous = start;

  while ((uint32_t)(micros() - start) < WINDOW_US) {
    const uint32_t now = micros();
    const uint16_t elapsed = (uint16_t)(now - previous);
    previous = now;

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      if (digitalRead(SENSOR_PINS[i]) == LOW) {
        readings[i] += elapsed;
      }
    }
  }
}

bool ballDetected() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (readings[i] >= DETECT_MIN_LOW_US) {
      return true;
    }
  }

  return false;
}

void updateFilter() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (!filterInitialized) {
      filteredReadings[i] = readings[i];
    } else {
      filteredReadings[i] = (filteredReadings[i] * 3UL + readings[i]) / 4;
    }
  }

  filterInitialized = true;
}

uint8_t getFilteredStrongestSensorIndex() {
  uint8_t strongest = lastSensorIndex;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (filteredReadings[i] > filteredReadings[strongest]) {
      strongest = i;
    }
  }

  return strongest;
}

uint8_t getStableSensorIndex() {
  const uint8_t candidate = getFilteredStrongestSensorIndex();

  if (candidate == lastSensorIndex) {
    pendingCount = 0;
    return lastSensorIndex;
  }

  if (filteredReadings[candidate] < filteredReadings[lastSensorIndex] + SWITCH_MARGIN_US) {
    pendingCount = 0;
    return lastSensorIndex;
  }

  if (candidate == pendingSensorIndex) {
    ++pendingCount;
  } else {
    pendingSensorIndex = candidate;
    pendingCount = 1;
  }

  if (pendingCount >= SWITCH_CONFIRMATIONS) {
    lastSensorIndex = candidate;
    pendingCount = 0;
  }

  return lastSensorIndex;
}

void seguir_bola_simples() {
  switch (direcaoIrAtual) {
    case IR_FRENTE:
      move_foward(velocidadeFrente);
      break;

    case IR_FRENTE_DIREITA:
      curva_frente_direita(velocidadeFrente, correcaoCurvaFrente);
      break;

    case IR_DIREITA:
    case IR_TRAS_DIREITA:
    case IR_TRAS:
      right_rotation(velocidadeGiro);
      break;

    case IR_TRAS_ESQUERDA:
    case IR_ESQUERDA:
      left_rotation(velocidadeGiro);
      break;

    case IR_FRENTE_ESQUERDA:
      curva_frente_esquerda(velocidadeFrente, correcaoCurvaFrente);
      break;

    case IR_NAO_DETECTADA:
    default:
      right_rotation(velocidadeBusca);
      break;
  }
}

void monitor_serial() {
  if (!DEBUG_SERIAL_ATIVO) {
    return;
  }

  if (millis() - ultimoPrintSerial < 500) {
    return;
  }

  ultimoPrintSerial = millis();

  Serial.print("Sensor IR: ");
  if (sensorIrAtual == 0) {
    Serial.print("NAO DETECTADO");
  } else {
    Serial.print(sensorIrAtual);
  }

  Serial.print(" / Direcao: ");
  Serial.println(directionName[direcaoIrAtual]);
}

void motor_1(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_3_ph_1, HIGH);
    digitalWrite(in_4_ph_1, LOW);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_3_ph_1, LOW);
    digitalWrite(in_4_ph_1, HIGH);
  } else {
    digitalWrite(in_3_ph_1, LOW);
    digitalWrite(in_4_ph_1, LOW);
  }

  analogWrite(pwm_1, speed);
}

void motor_2(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_1_ph_1, HIGH);
    digitalWrite(in_2_ph_1, LOW);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_1_ph_1, LOW);
    digitalWrite(in_2_ph_1, HIGH);
  } else {
    digitalWrite(in_1_ph_1, LOW);
    digitalWrite(in_2_ph_1, LOW);
  }

  analogWrite(pwm_2, speed);
}

void motor_3(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_3_ph_2, HIGH);
    digitalWrite(in_4_ph_2, LOW);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_3_ph_2, LOW);
    digitalWrite(in_4_ph_2, HIGH);
  } else {
    digitalWrite(in_3_ph_2, LOW);
    digitalWrite(in_4_ph_2, LOW);
  }

  analogWrite(pwm_3, speed);
}

void motor_4(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_1_ph_2, LOW);
    digitalWrite(in_2_ph_2, HIGH);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_1_ph_2, HIGH);
    digitalWrite(in_2_ph_2, LOW);
  } else {
    digitalWrite(in_1_ph_2, LOW);
    digitalWrite(in_2_ph_2, LOW);
  }

  analogWrite(pwm_4, speed);
}

void move_foward(int v) {
  motor_1(ANTI_HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(HORARIO, v);
  motor_4(ANTI_HORARIO, v);
}

void move_back(int v) {
  motor_1(HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(HORARIO, v);
}

void move_right(int v) {
  motor_1(ANTI_HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(HORARIO, v);
  motor_4(HORARIO, v);
}

void move_left(int v) {
  motor_1(HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(ANTI_HORARIO, v);
}

void curva_frente_esquerda(int velocidadeBase, int correcao) {
  int velocidadeInterna = constrain(velocidadeBase - correcao, 0, 255);
  int velocidadeExterna = constrain(velocidadeBase, 0, 255);

  motor_1(ANTI_HORARIO, velocidadeExterna);
  motor_2(ANTI_HORARIO, velocidadeInterna);
  motor_3(HORARIO, velocidadeInterna);
  motor_4(ANTI_HORARIO, velocidadeExterna);
}

void curva_frente_direita(int velocidadeBase, int correcao) {
  int velocidadeInterna = constrain(velocidadeBase - correcao, 0, 255);
  int velocidadeExterna = constrain(velocidadeBase, 0, 255);

  motor_1(ANTI_HORARIO, velocidadeInterna);
  motor_2(ANTI_HORARIO, velocidadeExterna);
  motor_3(HORARIO, velocidadeExterna);
  motor_4(ANTI_HORARIO, velocidadeInterna);
}

void left_rotation(int v) {
  motor_1(ANTI_HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(ANTI_HORARIO, v);
}

void right_rotation(int v) {
  motor_1(HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(HORARIO, v);
  motor_4(HORARIO, v);
}

void stop_robot() {
  motor_1(PARADO, 0);
  motor_2(PARADO, 0);
  motor_3(PARADO, 0);
  motor_4(PARADO, 0);
}
