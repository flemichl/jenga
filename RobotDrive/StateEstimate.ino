//void StateEstimate() {
//  // Caluclate velocity
//  float dl = (EncCountL - EncCountL_Last) / PULSES_CM; // change in encoder counts
//  float dr = (EncCountR - EncCountR_Last) / PULSES_CM;
//  float V_L = (float)dl / dt; // numerically calculate velocity
//  float V_R = (float)dr / dt;
//  Serial.print("deltaL: "); Serial.println(dl); Serial.print("\t");
//  Serial.print("deltaR: "); Serial.println(dr); Serial.print("\n");
//  Serial.print("VelR: "); Serial.println(V_L); Serial.print("\t");
//  Serial.print("VelL: "); Serial.println(V_R); Serial.print("\n");
//
//  // Calculate Omega for robot
//  float Omega = (V_R - V_L) / (float) WHEEL_BASE;
//
//  // Update Theta
//  float dTheta = Omega * dt;
//  Theta = Theta + dTheta;
//  
//  // Calcuate radius instantaneous center of rotation
//  float R = (float)WHEEL_BASE / 2.0 * (V_L + V_R) / (V_L + V_R);
//
//  // Save last position readings
//  float X_pos_last = X_pos;
//  float Y_pos_last = Y_pos;
//  
//  // Calculate Instantaneous Center of Roatation
//  float ICC_X = X_pos - R * sin(Theta);
//  float ICC_Y = Y_pos + R * cos(Theta);
//  
//  // Calculate theta relative to some absolute position.  Useful for keeping global x and y
//  float dx_ICC = X_pos - ICC_X;
//  float dy_ICC = Y_pos - ICC_Y;
//  X_pos = X_pos_last + dx_ICC * cos(dTheta) - dy_ICC * sin(dTheta);
//  Y_pos = Y_pos_last + dx_ICC * sin(dTheta) + dy_ICC * cos(dTheta);
//
//  Serial.print("X Position: ");Serial.print(X_pos); Serial.print("\t");
//  Serial.print("Y Position: ");Serial.print(Y_pos); Serial.print("\n");
//  PrintT("X Position", X_pos);
//}                                                          
