void test_wheelbase(){
  // drive straight
  targetDistanceL[0] = 70;
  targetDistanceR[0] = 70;

  // turn 90 around Right Wheel
  targetDistanceL[1] = WHEEL_BASE * PI_VALUE / 2;
  targetDistanceR[1] = 0;

  // drive straight
  targetDistanceL[2] = 20;
  targetDistanceR[2] = 20;

  // turn 180 around Left Wheel
  targetDistanceL[3] = 0;
  targetDistanceR[3] = WHEEL_BASE * PI_VALUE;
}

void demo1() {
  targetDistanceL[0] = TARGET_DISTANCE;
  targetDistanceR[0] = TARGET_DISTANCE;

  targetDistanceL[1] = TARGET_DISTANCE;
  targetDistanceR[1] = -1 * TARGET_DISTANCE;

  targetDistanceL[2] = TARGET_DISTANCE;
  targetDistanceR[2] = TARGET_DISTANCE;

  targetDistanceL[3] = -1 * TARGET_DISTANCE;
  targetDistanceR[3] = TARGET_DISTANCE;

  nextTargetIdx = 0;
}
