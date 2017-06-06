void forwardL(int speed_percent) {
//  int speed_pwm = map(speed_percent, SPEED_MIN, SPEED_MAX, PWM_MIN_L, PWM_MAX_L);
  int speed_pwm = speed_percent;
  analogWrite(ENA, speed_pwm);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void backwardL(int speed_percent) {
//  int speed_pwm = map(speed_percent, SPEED_MIN, SPEED_MAX, PWM_MIN_L, PWM_MAX_L);
  int speed_pwm = speed_percent;
  analogWrite(ENA, speed_pwm);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void forwardR(int speed_percent) {
//  int speed_pwm = map(speed_percent, SPEED_MIN, SPEED_MAX, PWM_MIN_R, PWM_MAX_R);
  int speed_pwm = speed_percent;
  analogWrite(ENB, speed_pwm);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backwardR(int speed_percent) {
//  int speed_pwm = map(speed_percent, SPEED_MIN, SPEED_MAX, PWM_MIN_R, PWM_MAX_R);
  int speed_pwm = speed_percent;
  analogWrite(ENB, speed_pwm);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void halt() {
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);        //left wheel holds still
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);        // right wheel holds still
}
// Move left wheel - accepts negative speeds
void LeftMove(int speedPWM) {
  if (speedPWM >= 0) {
    forwardL(speedPWM);
  }
  else if (speedPWM <= 0) {
    backwardL(abs(speedPWM));
  }
  else {
    Serial.println("Error. Left PWM");
  }
}
// Move Right Wheel - accepts negative speeds
void RightMove(int speedPWM) {
  if (speedPWM >= 0) {
    forwardR(speedPWM);
  }
  else if (speedPWM <= 0) {
    backwardR(abs(speedPWM));
  }
  else {
    Serial.println("Error. Right PWM");
  }
}

int clampPWM2(int speedPWM, int minPWM, int maxPWM) {
  if (speedPWM >= 0) {
    return min(maxPWM, max(minPWM, speedPWM));
  }
  else {
    return -1 * min(maxPWM, max(minPWM, abs(speedPWM)));
  }
}

//int mapPWMSpeed(int speed_percent, int SMin, int SMax, int PWM_Min, int PWM_Max) {
//  int out_speed_pwm;
//  if (speed_percent >= 0) {
//    out_speed_pwm = map(speed_percent, SMin, SMax, PWM_Min, PWM_Maxn);
//  }
//  else (speed_percent < 0) {
//    out_speed_pwm = -1 * map(abs(speed_percent), SMin, SMax, PWM_Min, PWM_Max);
//  }
//  return out_speed_pwm;
//}

