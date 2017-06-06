float Vel_Des, W_Des;

void Straight(float distance) {
  // set velocity based on direction (forward or reverse)
  if (distance < 0) {
    Vel_Des = -2.0;
  }
  else if ( distance > 0) {
    Vel_Des = 2.0;
  }
  W_Des = 0;
  
  float x, x_des, tol, theta, y, y_des;
  x = 0; theta = 0; y = 0;
  x_des = distance / WHEEL_CIRCUMFERENCE * PULSES_REVOLUTION;   y_des = 0;
  tol = 100; 

  float L_errorT, R_errorT;
  
  Serial.print("Starting Test:  Distance = "); Serial.println(x_des);
  unsigned long start_time, end_time, loop_time;
  start_time = millis();
  loop_time = start_time;
  int16_t RW_start, RW_end;
  int16_t LW_start, LW_end;
  unsigned long dt_loop;
  RW_start = RightWheelEncoder.read(); // count (int);
  LW_start = LeftWheelEncoder.read(); // count (int);
  int Lpwm; int Rpwm;
  if (Vel_Des > 0) { // may want to change this proportional to distance (like some max value if dist is large, otherwise, some smaller value!)
    Lpwm = 50; Rpwm = 50;
  }
  else if (Vel_Des < 0) {
    Lpwm = -50; Rpwm = -50;
  }
  while ((fabs(x_des - x) > tol) && teleop_stop_flag == 0) {
    SerialRead(); // check communication and change values as necessary
    // move at some constant pwm
    LeftMove(Lpwm);
    RightMove(Rpwm);
    end_time = millis();
    if ((end_time - start_time) >= 100) { // ensure sufficient time between readings
      RW_end = RightWheelEncoder.read();
      LW_end = LeftWheelEncoder.read();
      dt_loop = (unsigned long)(end_time - start_time);

      // send message
      Serial.print("S1 "); // start line
      PrintS("dt", dt_loop); PrintS("RW_Start", RW_start); PrintS("LW_Start", LW_start);
      PrintS("RW_End", RW_end); PrintS("LW_End", LW_end);
      PrintS("Time", (millis() - loop_time));
      PrintS("Rpwm", Rpwm); PrintS("Lpwm", Lpwm);
      Serial.println(); // end line marker

      // do control stuff
      float Vel_R = (float)(RW_end - RW_start) / dt_loop;
      float Vel_L = (float)(LW_end - LW_start) / dt_loop;
      float omega = (Vel_R - Vel_L) / (WHEEL_BASE / WHEEL_CIRCUMFERENCE * PULSES_REVOLUTION );
      theta += omega * dt_loop;
      float omega_error = 0.0 - omega; // omega should always be zero
      float kP = 2;
      float kP_w = 8;
      Rpwm += (Vel_Des - Vel_R) * kP + kP_w * (omega_error);
      Lpwm += (Vel_Des - Vel_L) * kP - kP_w * (omega_error);

      // update position
      x += 0.5 * (Vel_R + Vel_L) * dt_loop * cos(theta);
      y += 0.5 * (Vel_R + Vel_L) * dt_loop * sin(theta);
    
      // update values
      start_time = end_time;
      RW_start = RW_end; LW_start = LW_end;
    }
  }
  unsigned long halt_start = millis();
  Serial.println("Ending Path");  // need this when trying to record data!
  while ((millis() - halt_start) < 1000) { // make sure the robot comes to a complete stop before doing whatever hes going to do next
    halt();
    Vel_Des = 0; W_Des = 0; teleop_stop_flag = 1;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
void Rotation(float rot) {
  if (rot < 0) { // change W_Des based on theta?
    W_Des = -0.5;
  }
  else {
    W_Des = 0.5;
  }
  Vel_Des = 0;
  
  float x, x_des, tol, theta, y, y_des, theta_des, tol_theta;
  x = 0; x_des = 0; tol = 100; theta = 0; y = 0; y_des = 0; tol_theta = 0.05; theta_des = rot * (2); // the 2-scale factor I don't know why? but it works ok
  Serial.print("Starting Test:  Rotation = "); Serial.println(theta_des);
      
  unsigned long start_time, end_time, loop_time;
  start_time = millis();
  loop_time = start_time;
  int16_t RW_start, RW_end;
  int16_t LW_start, LW_end;
  unsigned long dt_loop;
  RW_start = RightWheelEncoder.read(); // count (int);
  LW_start = LeftWheelEncoder.read(); // count (int);
  int Lpwm; int Rpwm;
  if (W_Des > 0) {
    Lpwm = -40; Rpwm = 40;
  }
  else if (W_Des < 0) {
    Lpwm = 40; Rpwm = -50;
  }
  while (fabs(theta_des - theta) > tol_theta && teleop_stop_flag == 0) {
    SerialRead(); // check communication and change values as necessary
    // move at some constant pwm
    LeftMove(Lpwm);
    RightMove(Rpwm);
    delay(10);
    end_time = millis();
    if ((end_time - start_time) >= 100) { // ensure sufficient time between readings
      RW_end = RightWheelEncoder.read();
      LW_end = LeftWheelEncoder.read();
      dt_loop = (unsigned long)(end_time - start_time);
  
      // add controller if that is your thing
      float Vel_R = (float)(RW_end - RW_start) / dt_loop;
      float Vel_L = (float)(LW_end - LW_start) / dt_loop;
      float omega = (Vel_R - Vel_L) / (WHEEL_BASE / WHEEL_CIRCUMFERENCE * PULSES_REVOLUTION);
      /////// this is if the gyro were to be added.
//      float omega_enc = (Vel_R - Vel_L) / (WHEEL_BASE / WHEEL_CIRCUMFERENCE * PULSES_REVOLUTION);
//      float omega_gyro = GyroReading();
//      float alpha = 0.8; // relative value of readings
//      float omega;
//      if (fabs(omega_enc - omega_gyro) > 3) {
//        Serial.println("Going with Gyro Only");
//        omega = omega_gyro;
//      }
//      else {
//        omega = omega_enc * alpha + omega_gyro * (1 - alpha);
//      }

      theta += omega * dt_loop;
  //        Serial.println((double)(y_des - y));
  //        float theta_des = atan2((double)(y_des - y), (double)(x_des - x));
      float omega_error;
      omega_error = W_Des - omega; 
      float kP = 2;
      float kP_w = 4;
      Serial.print("Omega: ");
      Serial.println(omega_error);
      if (Vel_Des == 0) {
        Rpwm += kP_w * (omega_error);
        Lpwm -= kP_w * (omega_error);
      }
      else {
        Rpwm += (Vel_Des - Vel_R) * kP + kP_w * (omega_error);
        Lpwm += (Vel_Des - Vel_L) * kP - kP_w * (omega_error);
      }


      // send message
      Serial.print("S1 "); // start line
      PrintS("dt", dt_loop); PrintS("RW_Start", RW_start); PrintS("LW_Start", LW_start);
      PrintS("RW_End", RW_end); PrintS("LW_End", LW_end);
      PrintS("Time", (millis() - loop_time));
      PrintS("Rpwm", Rpwm); PrintS("Lpwm", Lpwm);
//      PrintS("W_Enc", omega_enc); PrintS("W_Gyro", omega_gyro);
      Serial.println(); // end line marker


      
      // update position -- this is completely wrong.  need to have diff drive kinematics
      x += 0.5 * (Vel_R + Vel_L) * dt_loop * cos(theta);
      y += 0.5 * (Vel_R + Vel_L) * dt_loop * sin(theta);
      
      // update values
      start_time = end_time;
      RW_start = RW_end; LW_start = LW_end;

      
    }
  }
  unsigned long halt_start = millis();
  Serial.println("Ending Path");  // need this when trying to record data!
  while ((millis() - halt_start) < 1000) { // make sure the robot comes to a complete stop before doing whatever hes going to do next
    halt();
    Vel_Des = 0; W_Des = 0;
  }
}



