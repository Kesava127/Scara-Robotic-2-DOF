#include <math.h>
#include <stdlib.h>
#include <string.h>

// CNC Shield V3 pins
const uint8_t EN=8, LEFT_DIR=5, RIGHT_DIR=6, Z_DIR=7;
const uint8_t LEFT_STEP=2, RIGHT_STEP=3, Z_STEP=4;
const bool INVERT_LEFT=false, INVERT_RIGHT=false;
const float l1=100.0f, l2=100.0f, l3=100.0f, l4=100.0f, l5=50.0f;
const float STEPS_PER_DEGREE=8.888888889f;
const float HOME_X=50.0f, HOME_Y=100.0f;

// Tunables
const float X_MIN=-100.0f, X_MAX=200.0f, Y_MIN=5.0f, Y_MAX=195.0f;
const float MAX_SPEED=60.0f, MAX_ACCEL=120.0f;     // mm/s, mm/s^2
const float MAX_JOINT_STEP_RATE=700.0f;            // steps/s ceiling
const float MAX_JOINT_STEP_ACCEL=1800.0f;           // steps/s^2, joint pulse-rate ramp
const long BACKLOG_MAX=40;                         // max steps the motors may lag the trajectory
const uint32_t CONTROL_TIMEOUT_MS=250, TRAJECTORY_MS=5;
const uint8_t LINE_CAP=48;

float x=HOME_X, y=HOME_Y, vx=0, vy=0, requestedX=0, requestedY=0;
float theta1Reference=0, theta2Reference=0;
long jointStep[2]={0,0}, jointGoal[2]={0,0};
bool initialized=false, movementEnabled=true, haveFreshInput=false;
uint32_t lastCommandMs=0, lastTrajectoryMs=0, lastStepUs=0;
float jointStepRate=0;
char lineBuffer[LINE_CAP]; uint8_t lineLength=0; bool droppingLine=false;

bool inverseKinematics(float px,float py,float &theta1,float &theta2) {
  float aL=px,bL=py,cL=(l1*l1-l2*l2+px*px+py*py)/(2*l1);
  float dL=aL*aL+bL*bL-cL*cL;
  float aR=px-l5,bR=py;
  float cR=(l4*l4+l5*l5-l3*l3-2*px*l5+px*px+py*py)/(2*l4);
  float dR=aR*aR+bR*bR-cR*cR;
  if(dL < -0.0001f || dR < -0.0001f) return false;
  if(dL<0)dL=0;
  if(dR<0)dR=0;
  theta1=2*atan2(-bL-sqrt(dL),-aL-cL)*180.0f/PI;
  theta2=2*atan2(-bR+sqrt(dR),-aR-cR)*180.0f/PI;
  if(!isfinite(theta1)||!isfinite(theta2)) return false;   // check BEFORE the wrap loops
  while(theta1<0)theta1+=360;
  while(theta1>=360)theta1-=360;
  while(theta2<0)theta2+=360;
  while(theta2>=360)theta2-=360;
  return true;
}

void report(const char *msg) { Serial.println(msg); }
void rejectLine() { report("ERR,COMMAND"); }

bool parseFinite(const char *s,float &v) {
  if(!s||!*s)return false;
  // AVR Arduino cores may not expose strtof(). Use strtod(), which is
  // available in the AVR libc used by the supported boards, then validate
  // that the entire field was consumed and the result is finite.
  char *end=0; double parsed=strtod(s,&end);
  if(!end || end==s || *end!='\0' || !isfinite(parsed)) return false;
  v=(float)parsed;
  return isfinite(v);
}

void setVelocity(float jx,float jy) {
  if(!movementEnabled || !initialized) { requestedX=requestedY=0; return; }
  requestedX=jx*MAX_SPEED; requestedY=jy*MAX_SPEED;
  lastCommandMs=millis(); haveFreshInput=true;
}

void handleLine(char *s) {
  while(*s==' '||*s=='\t')s++;
  if(!strcmp(s,"STOP")) { requestedX=requestedY=0; haveFreshInput=false; report("OK,STOP"); return; }
  if(!strcmp(s,"DISABLE")) {
    movementEnabled=false; requestedX=requestedY=0; haveFreshInput=false;
    vx=vy=0; jointGoal[0]=jointStep[0]; jointGoal[1]=jointStep[1];
    digitalWrite(EN,HIGH); report("OK,DISABLE"); return;
  }
  if(!strcmp(s,"ENABLE")) {
    movementEnabled=true; digitalWrite(EN,LOW);
    requestedX=requestedY=0; vx=vy=0; haveFreshInput=false;
    jointGoal[0]=jointStep[0]; jointGoal[1]=jointStep[1];
    report("OK,ENABLE"); return;
  }
  if(!strcmp(s,"STATUS")) {
    Serial.print("STATUS,");Serial.print(x,2);Serial.print(',');Serial.print(y,2);Serial.print(',');
    Serial.print(vx,2);Serial.print(',');Serial.print(vy,2);Serial.print(',');
    Serial.println(movementEnabled?"ENABLED":"DISABLED"); return;
  }
  if(!strncmp(s,"JOY,",4)) {
    char *comma=strchr(s+4,','); if(!comma){rejectLine();return;}
    *comma='\0';
    float jx,jy;
    if(!parseFinite(s+4,jx)||!parseFinite(comma+1,jy)||fabs(jx)>1.0f||fabs(jy)>1.0f){rejectLine();return;}
    setVelocity(jx,jy); return;
  }
  rejectLine();
}

// Bounded, non-blocking line receiver; overlong lines are discarded through newline.
void serviceSerial() {
  while(Serial.available()>0) {
    char c=(char)Serial.read();
    if(c=='\n') {
      if(!droppingLine && lineLength>0) { lineBuffer[lineLength]='\0'; handleLine(lineBuffer); }
      lineLength=0; droppingLine=false;
    } else if(c!='\r') {
      if(!droppingLine) {
        if(lineLength<LINE_CAP-1) lineBuffer[lineLength++]=c;
        else droppingLine=true;
      }
    }
  }
}

float approach(float current,float target,float maxDelta) {
  if(target>current+maxDelta)return current+maxDelta;
  if(target<current-maxDelta)return current-maxDelta;
  return target;
}

float wrap180(float d) {
  while(d>180)d-=360;
  while(d<-180)d+=360;
  return d;
}

void serviceTrajectory(uint32_t now) {
  if((uint32_t)(now-lastTrajectoryMs)<TRAJECTORY_MS)return;
  float dt=(now-lastTrajectoryMs)/1000.0f; if(dt<=0)dt=0.005f;
  lastTrajectoryMs=now;

  if(!haveFreshInput || (uint32_t)(now-lastCommandMs)>CONTROL_TIMEOUT_MS) requestedX=requestedY=0;

  // Accelerate toward requested velocity with a limit on the velocity-vector change.
  float dvx=requestedX-vx, dvy=requestedY-vy;
  float mag=sqrt(dvx*dvx+dvy*dvy), lim=MAX_ACCEL*dt;
  if(mag>lim && mag>0){ vx+=dvx*lim/mag; vy+=dvy*lim/mag; }
  else { vx=requestedX; vy=requestedY; }

  // FIX: don't let the commanded position run ahead of the motors. Without this,
  // x/y drift away from the real arm position whenever joint speed exceeds the step ceiling.
  long backlog=max(labs(jointGoal[0]-jointStep[0]), labs(jointGoal[1]-jointStep[1]));
  if(backlog>BACKLOG_MAX) return;

  float tx=x+vx*dt, ty=y+vy*dt;
  tx=fmax(X_MIN,fmin(X_MAX,tx)); ty=fmax(Y_MIN,fmin(Y_MAX,ty));

  float t1,t2;
  if(!inverseKinematics(tx,ty,t1,t2)) {
    float lo=0,hi=1;
    for(uint8_t i=0;i<10;i++){
      float f=(lo+hi)*0.5f;
      if(inverseKinematics(x+(tx-x)*f,y+(ty-y)*f,t1,t2))lo=f; else hi=f;
    }
    tx=x+(tx-x)*lo; ty=y+(ty-y)*lo;
    if(lo<0.01f || !inverseKinematics(tx,ty,t1,t2)){vx=vy=0;return;}
  }

  jointGoal[0]=lroundf(wrap180(t1-theta1Reference)*STEPS_PER_DEGREE);
  jointGoal[1]=lroundf(wrap180(t2-theta2Reference)*STEPS_PER_DEGREE);
  x=tx; y=ty;
}

// Coordinated Bresenham stepping at a fixed rate. Smoothness comes from the
// Cartesian accel limit; the old per-call "ramp" ran every loop() pass (not per
// time), so it reached full rate instantly and reset to 0 on every micro-pause.
void serviceSteppers(uint32_t nowUs) {
  static uint32_t rateUpdateUs=0;
  uint32_t rateDtUs=nowUs-rateUpdateUs; rateUpdateUs=nowUs;
  float rateDelta=MAX_JOINT_STEP_ACCEL*(rateDtUs/1000000.0f);
  long d0=jointGoal[0]-jointStep[0], d1=jointGoal[1]-jointStep[1];
  long a0=labs(d0), a1=labs(d1), major=max(a0,a1);
  if(major==0) { jointStepRate=approach(jointStepRate,0,rateDelta); return; }
  jointStepRate=approach(jointStepRate,MAX_JOINT_STEP_RATE,rateDelta);
  uint32_t interval=(uint32_t)(1000000.0f/fmaxf(1.0f,jointStepRate));
  if((uint32_t)(nowUs-lastStepUs)<interval) return;
  lastStepUs=nowUs;

  static long previousGoal0=0, previousGoal1=0, err0=0, err1=0;
  if(previousGoal0!=jointGoal[0]||previousGoal1!=jointGoal[1]){
    previousGoal0=jointGoal[0]; previousGoal1=jointGoal[1]; err0=err1=0;
  }
  err0+=a0; err1+=a1;
  bool p0=err0>=major, p1=err1>=major;
  if(p0)err0-=major;
  if(p1)err1-=major;

  // Set direction first, give the driver setup time, then pulse STEP.
  if(p0)digitalWrite(LEFT_DIR,((d0>0)!=INVERT_LEFT)?HIGH:LOW);
  if(p1)digitalWrite(RIGHT_DIR,((d1>0)!=INVERT_RIGHT)?HIGH:LOW);
  if(p0||p1) delayMicroseconds(5);
  if(p0){digitalWrite(LEFT_STEP,HIGH); jointStep[0]+=(d0>0?1:-1);}
  if(p1){digitalWrite(RIGHT_STEP,HIGH); jointStep[1]+=(d1>0?1:-1);}
  delayMicroseconds(4);
  if(p0)digitalWrite(LEFT_STEP,LOW);
  if(p1)digitalWrite(RIGHT_STEP,LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(EN,OUTPUT); pinMode(LEFT_DIR,OUTPUT); pinMode(RIGHT_DIR,OUTPUT);
  pinMode(LEFT_STEP,OUTPUT); pinMode(RIGHT_STEP,OUTPUT); pinMode(Z_DIR,OUTPUT); pinMode(Z_STEP,OUTPUT);
  digitalWrite(LEFT_STEP,LOW); digitalWrite(RIGHT_STEP,LOW); digitalWrite(Z_STEP,LOW);
  digitalWrite(EN,LOW);
  initialized=inverseKinematics(x,y,theta1Reference,theta2Reference);
  jointStep[0]=jointGoal[0]=jointStep[1]=jointGoal[1]=0;
  Serial.println("Five-bar joystick controller ready; software referenced position only.");
  Serial.println("Physically align robot at X=50,Y=100 mm before enabling motion.");
  if(!initialized)Serial.println("ERR,HOME_IK");
  lastTrajectoryMs=millis(); lastCommandMs=millis(); lastStepUs=micros();
}

void loop() {
  serviceSerial();
  serviceTrajectory(millis());
  serviceSteppers(micros());
}
