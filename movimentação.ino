#define in_1_ph_1 25
#define in_2_ph_1 24
#define in_3_ph_1 23
#define in_4_ph_1 22

#define in_1_ph_2 47
#define in_2_ph_2 46
#define in_3_ph_2 49
#define in_4_ph_2 48

#define pwm_1 2
#define pwm_2 3
#define pwm_3 4
#define pwm_4 5

int velocidade = 160;

void setup()
{
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

  stop_robot();

  Serial.println("Teste automatico de movimentos");
  delay(2000);
}

void loop()
{
  Serial.println("FRENTE");
  move_forward();
  delay(3000);
  stop_robot();
  delay(1000);

  Serial.println("TRAS");
  move_back();
  delay(3000);
  stop_robot();
  delay(1000);

  Serial.println("DIAGONAL FRENTE DIREITA");
  diagonal_frente_direita();
  delay(3000);
  stop_robot();
  delay(1000);

  Serial.println("DIAGONAL FRENTE ESQUERDA");
  diagonal_frente_esquerda();
  delay(3000);
  stop_robot();
  delay(1000);

  Serial.println("DIAGONAL TRAS DIREITA");
  diagonal_tras_direita();
  delay(3000);
  stop_robot();
  delay(1000);

  Serial.println("DIAGONAL TRAS ESQUERDA");
  diagonal_tras_esquerda();
  delay(3000);
  stop_robot();
  delay(3000);
}

void motor_1(int estadoA, int estadoB, int speed)
{
  digitalWrite(in_3_ph_1, estadoA);
  digitalWrite(in_4_ph_1, estadoB);
  analogWrite(pwm_1, speed);
}

void motor_2(int estadoA, int estadoB, int speed)
{
  digitalWrite(in_1_ph_1, estadoA);
  digitalWrite(in_2_ph_1, estadoB);
  analogWrite(pwm_2, speed);
}

void motor_3(int estadoA, int estadoB, int speed)
{
  digitalWrite(in_3_ph_2, estadoA);
  digitalWrite(in_4_ph_2, estadoB);
  analogWrite(pwm_3, speed);
}

void motor_4(int estadoA, int estadoB, int speed)
{
  digitalWrite(in_1_ph_2, estadoA);
  digitalWrite(in_2_ph_2, estadoB);
  analogWrite(pwm_4, speed);
}

void move_forward()
{
  motor_1(LOW, HIGH, velocidade);
  motor_2(HIGH, LOW, velocidade);
  motor_3(HIGH, LOW, velocidade);
  motor_4(LOW, HIGH, velocidade);
}

void move_back()
{
  motor_1(HIGH, LOW, velocidade);
  motor_2(LOW, HIGH, velocidade);
  motor_3(LOW, HIGH, velocidade);
  motor_4(HIGH, LOW, velocidade);
}

void diagonal_frente_direita()
{
  motor_1(LOW, HIGH, velocidade);
  motor_2(LOW, LOW, 0);
  motor_3(HIGH, LOW, velocidade);
  motor_4(LOW, LOW, 0);
}

void diagonal_frente_esquerda()
{
  motor_1(LOW, LOW, 0);
  motor_2(HIGH, LOW, velocidade);
  motor_3(LOW, LOW, 0);
  motor_4(LOW, HIGH, velocidade);
}

void diagonal_tras_direita()
{
  motor_1(LOW, LOW, 0);
  motor_2(LOW, HIGH, velocidade);
  motor_3(LOW, LOW, 0);
  motor_4(HIGH, LOW, velocidade);
}

void diagonal_tras_esquerda()
{
  motor_1(HIGH, LOW, velocidade);
  motor_2(LOW, LOW, 0);
  motor_3(LOW, HIGH, velocidade);
  motor_4(LOW, LOW, 0);
}

void stop_robot()
{
  motor_1(LOW, LOW, 0);
  motor_2(LOW, LOW, 0);
  motor_3(LOW, LOW, 0);
  motor_4(LOW, LOW, 0);
}
