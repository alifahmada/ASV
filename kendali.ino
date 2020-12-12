float cariError(){
  float spYaw = bearing;
  float errorYaw = tuning[3]-heading;
  if(errorYaw>180) errorYaw-=360;
  if(errorYaw<-180) errorYaw+=360;
  return errorYaw;
}

int pid(float kp, float ki, float kd, float err){//50Hz
    float p, i, d, errorYaw, u,
          lastError;// error;
    int pwm;      
    const float iMax = 225;
    error = err;

//    if(error>180) error-=360;
//    if(error<-180) error+=360;
    
    if(error>-1 && error<1) error=0;
    p = kp * error;
    //if(-5<error<5){
      i = i+(ki*lastError)*elapsedTime;
      if(i>iMax) i = iMax;
      else if(i<-iMax) i = -iMax;
    //}
    //if(pwmYaw==servoYawMin)iYaw = -iYaw;
    d = kd*(error-lastError)/elapsedTime;
    lastError = error;
    
    u = p+i+d;
    if(u>675) u = 675;//125
    if(u<-675) u = -675;//125

    return u;
}

void gerak(){

  if((targetWp == 0 && direction == -1) || (targetWp + 1 == wpCount && direction == 1)){
    direction *= -1;
  }
  //targetWp = 1;
  targetWp += direction;  
  wpLat = WP[targetWp*2];
  wpLon = WP[targetWp*2+1];

  cek = false;
}

void algoritma(){
  if(cek){
  wpLat = WP[0];
  wpLon = WP[1];
  }
  
  jarak = vincenty().dis;
  bearing = vincenty().azimuth;
  
  if (bearing < 0 ) bearing += 360;

  if(Emergency == 0){
    if(jarak<1)gerak();
  }
  if(Emergency == 1) {
    cek = true;
  }
  
  pwmPid = pid(1.5,0.3,0.2, cariError());
  rudder = 1475 - pwmPid;
}

void remote(int chann, int inpin) {
  if (digitalRead(inpin) == HIGH) {
    rcStart[chann] = micros();
  }
  else {
    uint16_t rcComp = (uint16_t)(micros()-rcStart[chann]);
    rcShared[chann] = rcComp;
  }
}

void rcReadVal(){
  noInterrupts();
  memcpy(rcValue, (const void *)rcShared, sizeof(rcShared));
  interrupts();
}

void rem1() {
  remote(0, ch1);
}
void rem2() {
  remote(1, ch2);
}
void rem3() {
  remote(2, ch3);
}
void rem4() {
  remote(3, ch4);
}

//unsigned long readPulse(int pin){
//  static unsigned long risingTime;
//  static int lastState;
//  int state = digitalRead(pin);
//  unsigned long pulseLength = 0;
//
//  if(lastState == LOW && state ==HIGH){
//    unsigned long fallingTime = micros();
//    pulseLength = fallingTime - risingTime;
//  }
//  lastState = state;
//  return pulseLength;
//}
