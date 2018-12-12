#include <avr/io.h>

#define Kp 2
#define Ki 0.0025
#define Kd 110
#define velocidadeInicial 127

uint8_t sensor1, sensor2, sensor3, sensor4, sensor5;
int error = 0;
int P, I, D, PIDvalue, previousError, leftMotor, rightMotor, valorPID;

void initPWM(){
  DDRB |= (1<<1)|(1<<2)|(1<<3);
  DDRD |= (1<<5)|(1<<6)|(1<<3);
  PORTB |= (1<<1);
  PORTB &= ~(1<<2);
  PORTD &= ~(1<<5);
  PORTD |= (1<<6);
  TCCR2A=(1<<COM2B1) | (1<<COM2B0) |(1<<COM2A1) | (1<<COM2A0) | (1<<WGM21) | (1<<WGM20);
  TCCR2B=(1<<CS22) | (1<<CS21) | (1<<CS20);
  OCR2A=255;
  OCR2B=255;
}

char readError(){

    sensor1 = PIND & (1 << 2);
    sensor2 = PIND & (1 << 4);
    sensor3 = PIND & (1 << 7);
    sensor4 = PINB & (1 << 0);
    sensor5 = PINB & (1 << 4);

    if(!sensor1 && !sensor2 && !sensor3 && !sensor4 && sensor5){
      return error = 4;
    } else if (!sensor1 && !sensor2 && !sensor3 && sensor4 && sensor5){
      return error = 3;
    } else if (!sensor1 && !sensor2 && !sensor3 && sensor4 && !sensor5){
      return error = 2;
    } else if (!sensor1 && !sensor2 && sensor3 && sensor4 && !sensor5){
      return error = 1;
    } else if (!sensor1 && !sensor2 && sensor3 && !sensor4 && !sensor5){
      return error = 0;
    } else if (!sensor1 && sensor2 && sensor3 && !sensor4 && !sensor5){
      return error = -1;
    } else if (!sensor1 && sensor2 && !sensor3 && !sensor4 && !sensor5){
      return error = -2;
    } else if (sensor1 && sensor2 && !sensor3 && !sensor4 && !sensor5){
      return error = -3;
    } else if (sensor1 && !sensor2 && !sensor3 && !sensor4 && !sensor5){
      return error = -4;
    } else {
      return error = 0;
    }
}

char calculatePID ()

{
  error = readError();

  P = error;

  I += error;

  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);

  previousError = error;

  return PIDvalue;

}

int main(void) {
  initPWM();

  while(1){
    valorPID = calculatePID();
    leftMotor = velocidadeInicial - valorPID;
    rightMotor = velocidadeInicial + valorPID;
    

    if(leftMotor > 255){
      leftMotor = 255;
    }  else if(rightMotor > 255){
      rightMotor = 255;
    } else if (rightMotor < 0){
      rightMotor = 0;
    } else if (leftMotor < 0){
      leftMotor = 0;
    }

    OCR2A = leftMotor;
    OCR2B = rightMotor;
  }

}
