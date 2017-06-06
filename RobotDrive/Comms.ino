int teleop_stop_flag = 0; // when this goes to one, the user said to stop
int auto_drive_flag = 0;
int linear_dist;
float rot_dist;
void SerialRead() {
  char incomingByte = 'n';
  float d, theta;
  int drive_state = 0;
  int turn_state = 0;
  if (Serial.available()){
    incomingByte = Serial.read();
    if (incomingByte == 'f') {
      d = Serial.parseFloat();
      teleop_stop_flag = 0;
      Straight(d);
      auto_drive_flag = 0;
    }
    else if (incomingByte == 'b') {
      d = Serial.parseFloat();
      teleop_stop_flag = 0;
      Straight(-1 * d);
      auto_drive_flag = 0;
    }
    else if (incomingByte == 'L') { // turning left
      d = Serial.parseFloat();
      teleop_stop_flag = 0;
      Rotation(d);
      auto_drive_flag = 0;
    }
    else if (incomingByte == 'R') { // turning right
      d = Serial.parseFloat();
      teleop_stop_flag = 0;
      Rotation(-1 * d);
      auto_drive_flag = 0;
    }
    else if (incomingByte == 'i') {
      teleop_update_Vel(1);
      drive_state = 1;
      turn_state = 0;
      auto_drive_flag = 1;
      linear_dist = 100;
    }
    else if (incomingByte == ',') {
      teleop_update_Vel(-1);
      drive_state = 1;
      turn_state = 0;
      auto_drive_flag = 1;
      linear_dist = -100;
    }
    else if (incomingByte == 'k') {
      drive_state = 0;
      turn_state = 0;
      teleop_update_Vel(0);
      teleop_update_W(0);
      auto_drive_flag = 1;
    }
    else if (incomingByte == 'j') {
      teleop_update_W(1);
      drive_state = 0;
      turn_state = 1;
      auto_drive_flag = 1;
      rot_dist = 100;
    }
    else if (incomingByte == 'l') {
      teleop_update_W(-1);
      drive_state = 0;
      turn_state = 1;
      auto_drive_flag = 1;
      rot_dist = -100;
    }
    else {
      Serial.write("Not a Valid Input");
    }
  }

  // this part runs every time, so need a state variable
  if (auto_drive_flag == 1) {
    if (drive_state == 1) {
      Teleop_Straight(linear_dist);
    }
    else if (turn_state == 1) {
      Teleop_Rotation(rot_dist);
    }
  }
}

