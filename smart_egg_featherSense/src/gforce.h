

long retrieveMovement(float xNeutral, float yNeutral, float zNeutral) {
  // x is rotating around egg center (left/right)
  // y is rotating around egg middlle (top/down)
  // z is rotating around egg center (tumble)
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mma.getEvent(&accel, &gyro, &temp);
  float x1 = accel.acceleration.x;
  float y1 = accel.acceleration.y;
  float z1 = accel.acceleration.z;

  // Serial.print("X: ");
  // Serial.println(x1);
  // Serial.print("Y: ");
  // Serial.println(y1);
  // Serial.print("Z: ");
  // Serial.println(z1);

  return abs(x1+ y1 + z1);

}