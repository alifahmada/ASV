void bacaGps() {
  while (serialGps.available()) {
    gps.encode(serialGps.read());
  }
  if (gps.location.isUpdated()) {
    Lat = gps.location.lat();
    Lon = gps.location.lng();
    alt = gps.altitude.meters();
    sat = gps.satellites.value();
    v = gps.speed.kmph();
    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));

    //smartDelay(100);
  }
}

void initKompas(){
  mag.init();
  mag.enableDefault();
}

void bacaKompas(){
  mag.read();
  float offsetX, offsetY, offsetZ, xh, yh;

  float xmin = -3476;//-3698;
  float xmax = 1215;//1602;
  float ymin = -2342;//-1636;
  float ymax = 3674;//3608;
  float zmin = 986;//-1636;
  float zmax = 1312;//3608;

  float xsf = (ymax-ymin)/(xmax-xmin);
  float ysf = (xmax-xmin)/(ymax-ymin);
  
  offsetX = ((xmax-xmin)/2-xmax)*xsf;
  offsetY = ((ymax-ymin)/2-ymax)*ysf;
  offsetZ = (zmax-zmin)/2-zmax;
    
  compass[0] = (mag.m.x+offsetX)/6842;//x
  compass[1] = (mag.m.y+offsetY)/6842;//y
  compass[2] = mag.m.z+offsetZ;//z

  xh = compass[0]*cos(roll) + compass[1]*sin(pitch)*sin(roll) - compass[2]*cos(pitch)*sin(roll);
  yh = compass[1]*cos(pitch) + compass[2]*sin(pitch);
  
  float headingRad = atan2(compass[1], -compass[0]);//atan2(y,-x)
  //headingRad += 0.1825352228276; //ditambah sudut deklinasi
  if(headingRad > -1.5708){
    headingRad += 1.5708;
    //headingRad += 0.174533; //ditambah sudut deklinasi
  }
  if(headingRad < -1.5708){
    headingRad +=(2*M_PI)+1.5708;    
  }
  
  heading = deg(headingRad);
}

//void initMpu(){
//  while(!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G)){
//    Serial.println("Cek Koneksi MPU");
//    delay(500);
//  }
//  mpu.setAccelOffsetX(-44);
//  mpu.setAccelOffsetY(-2304);
//  mpu.setAccelOffsetZ(1204);
//  mpu.setGyroOffsetX(70);
//  mpu.setGyroOffsetY(-74);
//  mpu.setGyroOffsetZ(-46);
//}
//
//void bacaMpu(){
//  Vector normA = mpu.readNormalizeAccel();
//  Vector normG = mpu.readNormalizeGyro();
//
//  float KG= 0.98;
//  float KA= 0.02;
//  
//  accel[0] = normA.XAxis;//ax
//  accel[1] = normA.YAxis;//ay
//  accel[2] = normA.ZAxis;//az
//  gyro[0] = normG.XAxis;//gx
//  gyro[1] = normG.YAxis;//gy
//  gyro[2] = normG.ZAxis;//gz
//
//  gxr = gyro[0]/131;
//  gyr = gyro[1]/131;
//  gzr = gyro[2]/131;
//  
//  aPitch = deg((atan2(accel[0], sqrt(accel[1]*accel[1]+accel[2]*accel[2]))));//(atan2(ax, sqrt(ay*ay+az*az)))*rtod;//atan2(ax,ay)
//  aRoll = deg((atan2(accel[1], accel[2])));//atan2(ay, az)
//  gPitch = gPitch + (float)gxr *elapsedTime;
//  gRoll = gRoll + (float)gyr*elapsedTime;
//
//  float k = kalman.getAngle(aRoll, gxr, elapsedTime);
//  //float actPitch = (KA*aPitch) + (KG*(actPitch+(gxr*rtod*elapsedTime)));
//  //float actRoll = (KA*aRoll) + (KG*(actRoll+(gxr*rtod*elapsedTime)));
//
//  gPitch = (KG*gPitch) + (KA*aPitch);
//  gRoll = (KG*gRoll) + (KA*aRoll);;
//  
//  
//  pitch = gPitch;
//  roll = gRoll;
//}

void i2c_write(int address, int reg, int val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int i2c_read(int address, int reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, 1);
  int data = Wire.read();

  return data;
}

//void initKompas() {
//  i2c_write(0x1E, 0x20, 0b11011100);//ctrl_reg1
//  i2c_write(0x1E, 0x21, 0b01100000);
//  i2c_write(0x1E, 0x22, 0b00000000);
//  i2c_write(0x1E, 0x23, 0b00001000);
//  i2c_write(0x1E, 0x24, 0b11000000);
//}
//
//void bacaKompas() {
//  int status = i2c_read(0x1E, 0x27);
//  Wire.beginTransmission(0x1E);
//  Wire.write(0x28);
//  Wire.endTransmission();
//
//  if ((status & 0x08) == 0x08) {
//    cxl = i2c_read(0x1E, 0x28);
//    cxh = i2c_read(0x1E, 0x29);
//    cyl = i2c_read(0x1E, 0x2A);
//    cyh = i2c_read(0x1E, 0x2B);
//    czl = i2c_read(0x1E, 0x2C);
//    czh = i2c_read(0x1E, 0x2D);
//  }
//
//  cx = ((cxh << 8) | cxl); //* (0.0464f);
//  cy = ((cyh << 8) | cyl);//* (0.0464f);
//  cz = ((czh << 8) | czl);//* (0.0464f);
//
//  headingRad = atan2(cy, cx);
//
//  heading = headingRad * rtod;
//}
