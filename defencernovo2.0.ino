#include <Wire.h>
#include <HTInfraredSeeker.h>
#include <string.h>

// Motores e ponte H no padrao do primeiro codigo.
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

// MUX do sensor de linha do defender.
// Se seu MUX estiver nos pinos do primeiro codigo, troque para 34, 35, 36, 37 e SIG A0.
const int s0 = 8;
const int s1 = 9;
const int s2 = 10;
const int s3 = 11;
const int SIG_pin = A7;

const int NUM_SENSORES_LINHA = 16;

int valorMin[NUM_SENSORES_LINHA];
int valorMax[NUM_SENSORES_LINHA];
int limiar[NUM_SENSORES_LINHA];
bool sensorCalibrado[NUM_SENSORES_LINHA];
bool linha[NUM_SENSORES_LINHA];
bool sensoresDetectados[NUM_SENSORES_LINHA];
int leiturasLinha[NUM_SENSORES_LINHA];

const int TEMPO_CALIBRACAO = 6000;
const int VARIACAO_MINIMA = 80;
const bool LINHA_PRETA_VALOR_MENOR = true;

// Vetores adaptados para 16 sensores em volta do robo.
const float vectorX[NUM_SENSORES_LINHA] = {
   1.0000,  0.9239,  0.7071,  0.3827,
   0.0000, -0.3827, -0.7071, -0.9239,
  -1.0000, -0.9239, -0.7071, -0.3827,
   0.0000,  0.3827,  0.7071,  0.9239
};

const float vectorY[NUM_SENSORES_LINHA] = {
   0.0000,  0.3827,  0.7071,  0.9239,
   1.0000,  0.9239,  0.7071,  0.3827,
   0.0000, -0.3827, -0.7071, -0.9239,
  -1.0000, -0.9239, -0.7071, -0.3827
};

int lineAngle = 500;
int lineDepth = 0;
int lineSide = 0; // 0 = nenhum, 1 = direita, 3 = esquerda
bool linhaDetectada = false;

// Sensor IR da bola.
int ballDirecao = 0;
int ballIntens = 0;
const int BALL_CENTER = 5;

// Velocidades do goleiro.
const int FORWARD_SPEED = 160;
const int BACK_SPEED = 150;
const int SIDE_SPEED = 140;
const int SEARCH_SPEED = 105;
const int CORRECTION_SPEED = 115;

// Se a linha estiver muito profunda no robo, ele corrige.
const int PROFUNDIDADE_MAXIMA = 5;

const int HORARIO = 1;
const int ANTI_HORARIO = -1;
const int PARADO = 0;

bool habilitar_debug = true;
unsigned long ultimoPrintSerial = 0;
const unsigned long INTERVALO_SERIAL = 500;
const char* ultimoMovimento = "";
const char* ultimoEstado = "";

void setup() {
  Serial.begin(115200);

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

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(SIG_pin, INPUT);

  Wire.begin();
  InfraredSeeker::Initialize();

  stop_robot();

  Serial.println("=== DEFENDER COM LDR VETORIAL VIA MUX ===");
  calibrar_linha();
  Serial.println("Defender iniciado.");
}

void loop() {
  ir_reader();
  atualizar_linha_vetorial();
  monitor_serial();

  controle_goleiro_com_ldr();

  delay(25);
}

void controle_goleiro_com_ldr() {
  if (!linhaDetectada) {
    debug_msg("ESTADO: Linha perdida -> procurando para tras");
    move_back(SEARCH_SPEED);
    return;
  }

  if (lineSide == 1) {
    debug_msg("ESTADO: Limite direito -> corrigindo para esquerda");
    move_left(CORRECTION_SPEED);
    return;
  }

  if (lineSide == 3) {
    debug_msg("ESTADO: Limite esquerdo -> corrigindo para direita");
    move_right(CORRECTION_SPEED);
    return;
  }

  if (lineDepth >= PROFUNDIDADE_MAXIMA) {
    debug_msg("ESTADO: Linha muito profunda -> voltando");
    move_back(BACK_SPEED);
    return;
  }

  seguir_bola_eixo_x();
}

void seguir_bola_eixo_x() {
  if (ballDirecao == 0) {
    debug_msg("BOLA: Nao encontrada -> parado");
    stop_robot();
    return;
  }

  if (ballDirecao >= 4 && ballDirecao <= 6) {
    debug_msg("BOLA: Centralizada -> parado");
    stop_robot();
    return;
  }

  if (ballDirecao < BALL_CENTER) {
    debug_msg("BOLA: Esquerda -> mover esquerda");
    move_left(SIDE_SPEED);
    return;
  }

  if (ballDirecao > BALL_CENTER) {
    debug_msg("BOLA: Direita -> mover direita");
    move_right(SIDE_SPEED);
    return;
  }

  stop_robot();
}

void ir_reader() {
  InfraredResult bola = InfraredSeeker::ReadAC();
  ballDirecao = bola.Direction;
  ballIntens = bola.Strength;
}

void atualizar_linha_vetorial() {
  ler_linha_mux();
  calcular_vetor_linha();
  calcular_profundidade_linha();
  calcular_lado_linha();
}

void ler_linha_mux() {
  for (int i = 0; i < NUM_SENSORES_LINHA; i++) {
    leiturasLinha[i] = mediaLeituras(i);

    if (!sensorCalibrado[i]) {
      linha[i] = false;
      sensoresDetectados[i] = false;
      continue;
    }

    if (LINHA_PRETA_VALOR_MENOR) {
      linha[i] = leiturasLinha[i] <= limiar[i];
    } else {
      linha[i] = leiturasLinha[i] >= limiar[i];
    }

    sensoresDetectados[i] = linha[i];
  }
}

void calcular_vetor_linha() {
  float somaX = 0;
  float somaY = 0;
  int sensoresLendo = 0;

  for (int i = 0; i < NUM_SENSORES_LINHA; i++) {
    if (sensoresDetectados[i]) {
      somaX += vectorX[i];
      somaY += vectorY[i];
      sensoresLendo++;
    }
  }

  if (sensoresLendo == 0) {
    linhaDetectada = false;
    lineAngle = 500;
    lineDepth = 0;
    lineSide = 0;
    return;
  }

  linhaDetectada = true;
  lineAngle = atan2(somaY, somaX) * 180.0 / PI;

  if (lineAngle < 0) {
    lineAngle += 360;
  }
}

void calcular_profundidade_linha() {
  if (!linhaDetectada) {
    lineDepth = 0;
    return;
  }

  lineDepth = 1;

  for (int i = 0; i <= 7; i++) {
    int espelho = 15 - i;

    bool esquerda = sensoresDetectados[i];
    bool direita = sensoresDetectados[espelho];

    if (esquerda && direita) {
      lineDepth = i + 1;
      return;
    }

    if (esquerda) {
      if ((espelho > 0 && sensoresDetectados[espelho - 1]) ||
          (espelho < 15 && sensoresDetectados[espelho + 1])) {
        lineDepth = i + 1;
        return;
      }
    }

    if (direita) {
      if ((i > 0 && sensoresDetectados[i - 1]) ||
          (i < 15 && sensoresDetectados[i + 1])) {
        lineDepth = i + 1;
        return;
      }
    }
  }
}

void calcular_lado_linha() {
  lineSide = 0;

  if (!linhaDetectada) {
    return;
  }

  for (int i = 4; i <= 7; i++) {
    int espelho = 15 - i;

    bool esquerda = sensoresDetectados[i];
    bool direita = sensoresDetectados[espelho];

    if (esquerda && !direita) {
      lineSide = 3;
      return;
    }

    if (direita && !esquerda) {
      lineSide = 1;
      return;
    }
  }
}

void calibrar_linha() {
  Serial.println("Coloque o robo perto da linha.");
  Serial.println("Durante a calibracao, mova o robo para os sensores passarem na linha.");
  Serial.println("Digite 1 no Serial Monitor para comecar.");

  while (true) {
    if (Serial.available()) {
      int v = Serial.parseInt();
      if (v == 1) {
        break;
      }
    }
  }

  for (int i = 0; i < NUM_SENSORES_LINHA; i++) {
    valorMin[i] = 1023;
    valorMax[i] = 0;
    limiar[i] = 512;
    sensorCalibrado[i] = false;
    linha[i] = false;
    sensoresDetectados[i] = false;
  }

  Serial.println("Calibrando... mova o robo sobre a linha.");

  unsigned long inicio = millis();

  while (millis() - inicio < TEMPO_CALIBRACAO) {
    for (int i = 0; i < NUM_SENSORES_LINHA; i++) {
      int leitura = mediaLeituras(i);

      if (leitura < valorMin[i]) {
        valorMin[i] = leitura;
      }

      if (leitura > valorMax[i]) {
        valorMax[i] = leitura;
      }
    }

    delay(20);
  }

  Serial.println();
  Serial.println("=== RESULTADO CALIBRACAO ===");

  for (int i = 0; i < NUM_SENSORES_LINHA; i++) {
    int variacao = valorMax[i] - valorMin[i];
    limiar[i] = (valorMin[i] + valorMax[i]) / 2;
    sensorCalibrado[i] = variacao >= VARIACAO_MINIMA;

    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(" Min:");
    Serial.print(valorMin[i]);
    Serial.print(" Max:");
    Serial.print(valorMax[i]);
    Serial.print(" Var:");
    Serial.print(variacao);
    Serial.print(" L:");
    Serial.print(limiar[i]);
    Serial.print(" Status:");
    Serial.println(sensorCalibrado[i] ? "OK" : "SEM_VARIACAO");
  }
}

int mediaLeituras(int canal) {
  long soma = 0;

  for (int i = 0; i < 5; i++) {
    soma += readMux(canal);
    delay(2);
  }

  return soma / 5;
}

int readMux(int channel) {
  digitalWrite(s0, bitRead(channel, 0));
  digitalWrite(s1, bitRead(channel, 1));
  digitalWrite(s2, bitRead(channel, 2));
  digitalWrite(s3, bitRead(channel, 3));

  delayMicroseconds(50);

  return analogRead(SIG_pin);
}

void monitor_serial() {
  if (millis() - ultimoPrintSerial < INTERVALO_SERIAL) {
    return;
  }

  ultimoPrintSerial = millis();

  Serial.println("-------------------------------");
  Serial.print("Bola direcao: ");
  Serial.println(ballDirecao);
  Serial.print("Bola intensidade: ");
  Serial.println(ballIntens);
  Serial.print("Linha detectada: ");
  Serial.println(linhaDetectada ? "SIM" : "NAO");
  Serial.print("Angulo linha: ");
  Serial.println(lineAngle);
  Serial.print("Profundidade linha: ");
  Serial.println(lineDepth);
  Serial.print("Lado linha: ");
  Serial.println(nome_lado_linha(lineSide));
  imprimir_sensores_linha();
  Serial.println("-------------------------------");
}

void imprimir_sensores_linha() {
  Serial.print("Sensores: ");

  for (int i = 0; i < NUM_SENSORES_LINHA; i++) {
    if (sensoresDetectados[i]) {
      Serial.print("S");
      Serial.print(i + 1);
      Serial.print(" ");
    }
  }

  Serial.println();
}

const char* nome_lado_linha(int lado) {
  if (lado == 1) {
    return "DIREITA";
  }

  if (lado == 3) {
    return "ESQUERDA";
  }

  return "NENHUM";
}

void debug_msg(const char* msg) {
  if (habilitar_debug) {
    if (strcmp(msg, ultimoEstado) != 0) {
      Serial.println(msg);
      ultimoEstado = msg;
    }
  }
}

void print_movimento(const char* movimento) {
  if (!habilitar_debug) {
    return;
  }

  if (strcmp(movimento, ultimoMovimento) != 0) {
    Serial.print("MOVIMENTO: ");
    Serial.println(movimento);
    ultimoMovimento = movimento;
  }
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

void move_foward(int speed) {
  print_movimento("FRENTE");

  motor_1(ANTI_HORARIO, speed);
  motor_2(ANTI_HORARIO, speed);
  motor_3(HORARIO, speed);
  motor_4(ANTI_HORARIO, speed);
}

void move_back(int speed) {
  print_movimento("TRAS");

  motor_1(HORARIO, speed);
  motor_2(HORARIO, speed);
  motor_3(ANTI_HORARIO, speed);
  motor_4(HORARIO, speed);
}

void move_left(int speed) {
  print_movimento("ESQUERDA");

  motor_1(HORARIO, speed);
  motor_2(ANTI_HORARIO, speed);
  motor_3(ANTI_HORARIO, speed);
  motor_4(ANTI_HORARIO, speed);
}

void move_right(int speed) {
  print_movimento("DIREITA");

  motor_1(ANTI_HORARIO, speed);
  motor_2(HORARIO, speed);
  motor_3(HORARIO, speed);
  motor_4(HORARIO, speed);
}

void right_rotation(int speed) {
  print_movimento("ROTACIONAR DIREITA");

  motor_1(HORARIO, speed);
  motor_2(ANTI_HORARIO, speed);
  motor_3(HORARIO, speed);
  motor_4(HORARIO, speed);
}

void left_rotation(int speed) {
  print_movimento("ROTACIONAR ESQUERDA");

  motor_1(ANTI_HORARIO, speed);
  motor_2(HORARIO, speed);
  motor_3(ANTI_HORARIO, speed);
  motor_4(ANTI_HORARIO, speed);
}

void stop_robot() {
  print_movimento("PARADO");

  motor_1(PARADO, 0);
  motor_2(PARADO, 0);
  motor_3(PARADO, 0);
  motor_4(PARADO, 0);
}
