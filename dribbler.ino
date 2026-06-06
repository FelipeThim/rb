#include <Arduino.h>
#include <Servo.h>

#define DRIBBLER_PIN 14

Servo dribbler;

int pctParaMicro(int pct) {
    pct = constrain(pct, 0, 100);
    return map(pct, 0, 100, 1000, 2000);
}

void setup()
{
    dribbler.attach(DRIBBLER_PIN);

    // Arma o ESC
    dribbler.writeMicroseconds(1000);
    delay(3000);

    // Liga o dribbler
    dribbler.writeMicroseconds(pctParaMicro(35));
}

void loop()
{
    // Nada aqui por enquanto
}
