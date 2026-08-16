#include <HTInfraredSeeker.h>

// Pinos dos quatro motores de tracao.
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

// Multiplexador dos 16 LDRs.
#define S0 34
#define S1 35
#define S2 36
#define S3 37
#define SIG A0
#define EN 26

#define led_pin 6

#define HORARIO 1
#define ANTI_HORARIO -1
#define PARADO 0

const bool DEBUG_SERIAL_ATIVO = true;
const bool LINHA_VALOR_MENOR_QUE_QUADRA = true;
const int LIMIAR_LINHA = 60;

const int VELOCIDADE_GOLEIRO_LATERAL = 220;
const int VELOCIDADE_GOLEIRO_LATERAL_RAPIDA = 205;
const int VELOCIDADE_GOLEIRO_RE = 150;
const unsigned long INTERVALO_LOOP_MS = 10;

// Se a linha da frente for apenas referencia e nao limite, mude para false.
const bool GOLEIRO_AVANCAR_COM_LINHA_FRENTE = true;

int ballDirecao = 0;
int ldr[16];
int primeiroSensorLinha = -1;

bool linhaFrente = false;
bool linhaDireita = false;
bool linhaEsquerda = false;
bool linhaTraseira = false;

// Indices dos canais 0-15 do multiplexador.
// Frente: 14, 15, 0, 1, 2 (fisicamente 15, 16, 1, 2, 3).
// Direita: 11, 12, 13.
// Esquerda: 4, 5, 6. O canal 3 nao participa da logica de linha.
// Traseira: 7, 8, 9, com alertas 6 e 14.
const uint8_t LDR_FRENTE[] = {14, 15, 0, 1, 2};
const uint8_t LDR_DIREITA[] = {11, 12, 13};
const uint8_t LDR_ESQUERDA[] = {4, 5, 6};
const uint8_t LDR_TRASEIRA[] = {6, 7, 8, 9};

unsigned long ultimoDebug = 0;
const char* acaoAtual = "robo parado";

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

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG, INPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  InfraredSeeker::Initialize();
  stop_robot();
}

void loop() {
  ler_direcao_bola();
  atualizar_linhas();
  controlar_goleiro();
  mostrar_debug();
  delay(INTERVALO_LOOP_MS);
}

void ler_direcao_bola() {
  InfraredResult bola = InfraredSeeker::ReadAC();
  ballDirecao = bola.Direction;
}

void setMuxChannel(int canal) {
  digitalWrite(S0, canal & 0x01);
  digitalWrite(S1, (canal >> 1) & 0x01);
  digitalWrite(S2, (canal >> 2) & 0x01);
  digitalWrite(S3, (canal >> 3) & 0x01);
}

void ler_ldrs() {
  for (int i = 0; i < 16; i++) {
    setMuxChannel(i);
    delayMicroseconds(300);
    analogRead(SIG);
    ldr[i] = analogRead(SIG);
  }
}

bool ldr_esta_na_linha(int canal) {
  if (LINHA_VALOR_MENOR_QUE_QUADRA) {
    return ldr[canal] < LIMIAR_LINHA;
  }

  return ldr[canal] > LIMIAR_LINHA;
}

bool grupo_tem_linha(const uint8_t canais[], uint8_t totalCanais) {
  for (uint8_t i = 0; i < totalCanais; i++) {
    if (ldr_esta_na_linha(canais[i])) {
      if (primeiroSensorLinha == -1) {
        primeiroSensorLinha = canais[i];
      }
      return true;
    }
  }

  return false;
}

void atualizar_linhas() {
  ler_ldrs();
  primeiroSensorLinha = -1;

  linhaFrente = grupo_tem_linha(LDR_FRENTE, sizeof(LDR_FRENTE) / sizeof(LDR_FRENTE[0]));
  linhaDireita = grupo_tem_linha(LDR_DIREITA, sizeof(LDR_DIREITA) / sizeof(LDR_DIREITA[0]));
  linhaEsquerda = grupo_tem_linha(LDR_ESQUERDA, sizeof(LDR_ESQUERDA) / sizeof(LDR_ESQUERDA[0]));
  linhaTraseira = grupo_tem_linha(LDR_TRASEIRA, sizeof(LDR_TRASEIRA) / sizeof(LDR_TRASEIRA[0]));

  bool linhaDetectada = linhaFrente || linhaDireita || linhaEsquerda || linhaTraseira;
  digitalWrite(led_pin, linhaDetectada ? HIGH : LOW);
}

int velocidade_lateral() {
  if (ballDirecao == 1 || ballDirecao == 2 || ballDirecao == 8 || ballDirecao == 9) {
    return VELOCIDADE_GOLEIRO_LATERAL_RAPIDA;
  }

  return VELOCIDADE_GOLEIRO_LATERAL;
}

void controlar_goleiro() {
  // Linha traseira, incluindo os alertas fisicos 7 e 15, tem prioridade.
  // O goleiro da re ate a linha sair desse conjunto de LDRs.
  if (linhaTraseira) {
    move_back(VELOCIDADE_GOLEIRO_RE);
    acaoAtual = "robo dando re: linha traseira";
    return;
  }

  // Linha na frente reposiciona o robo para frente.
  if (GOLEIRO_AVANCAR_COM_LINHA_FRENTE && linhaFrente) {
    move_foward(VELOCIDADE_GOLEIRO_RE);
    acaoAtual = "robo indo para frente: linha frontal";
    return;
  }

  // O goleiro nao gira para procurar. Sem bola, ou com a bola ao centro, ele para.
  if (ballDirecao == 0 || ballDirecao == 5) {
    stop_robot();
    acaoAtual = ballDirecao == 0 ? "robo parado: sem bola" : "robo parado: bola ao centro";
    return;
  }

  int velocidade = velocidade_lateral();

  // Mesma interpretacao do HiTechnic usada no seu atacante.
  if (ballDirecao >= 1 && ballDirecao <= 4) {
    move_left(velocidade);
    acaoAtual = "robo indo para esquerda";
    return;
  }

  if (ballDirecao >= 6 && ballDirecao <= 9) {
    move_right(velocidade);
    acaoAtual = "robo indo para direita";
    return;
  }

  stop_robot();
  acaoAtual = "robo parado: direcao IR invalida";
}

void motor_1(int sentido, int velocidade) {
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
  analogWrite(pwm_1, velocidade);
}

void motor_2(int sentido, int velocidade) {
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
  analogWrite(pwm_2, velocidade);
}

void motor_3(int sentido, int velocidade) {
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
  analogWrite(pwm_3, velocidade);
}

void motor_4(int sentido, int velocidade) {
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
  analogWrite(pwm_4, velocidade);
}

void move_back(int velocidade) {
  motor_1(HORARIO, velocidade);
  motor_2(HORARIO, velocidade);
  motor_3(ANTI_HORARIO, velocidade);
  motor_4(HORARIO, velocidade);
}

void move_foward(int velocidade) {
  motor_1(ANTI_HORARIO, velocidade);
  motor_2(ANTI_HORARIO, velocidade);
  motor_3(HORARIO, velocidade);
  motor_4(ANTI_HORARIO, velocidade);
}

void move_right(int velocidade) {
  motor_1(ANTI_HORARIO, velocidade);
  motor_2(HORARIO, velocidade);
  motor_3(HORARIO, velocidade);
  motor_4(HORARIO, velocidade);
}

void move_left(int velocidade) {
  motor_1(HORARIO, velocidade);
  motor_2(ANTI_HORARIO, velocidade);
  motor_3(ANTI_HORARIO, velocidade);
  motor_4(ANTI_HORARIO, velocidade);
}

void stop_robot() {
  motor_1(PARADO, 0);
  motor_2(PARADO, 0);
  motor_3(PARADO, 0);
  motor_4(PARADO, 0);
}

void mostrar_debug() {
  if (!DEBUG_SERIAL_ATIVO || millis() - ultimoDebug < 250) {
    return;
  }

  ultimoDebug = millis();
  Serial.println("--------------------------------");
  Serial.print("IR Direcao: ");
  Serial.print(ballDirecao);
  Serial.print(" | Acao: ");
  Serial.println(acaoAtual);

  Serial.print("LDRs 1-16: ");
  for (int i = 0; i < 16; i++) {
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(ldr[i]);
    if (i < 15) {
      Serial.print(" | ");
    }
  }
  Serial.println();

  Serial.print(" | LDR F/D/E/T: ");
  Serial.print(linhaFrente);
  Serial.print("/");
  Serial.print(linhaDireita);
  Serial.print("/");
  Serial.print(linhaEsquerda);
  Serial.print("/");
  Serial.print(linhaTraseira);
  Serial.print(" | Primeiro: ");
  Serial.println(primeiroSensorLinha);
}
