#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <MAX30105.h>
#include "heartRate.h"

// ================= LCD =================
#define LCD_ADDR 0x27           // change to 0x3F if needed
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ================= GSR =================
#define GSR_PIN 33              // ADC1 pin (safe with WiFi)
const int   GSR_SMA = 16;       // smoothing window
const int   GSR_BASE_MS = 3000; // baseline duration on boot
float gsrBaseline = NAN;
int   gsrBuf[GSR_SMA]; int gsrIdx=0, gsrCount=0;

// ================= MAX3010x (PPG) =================
MAX30105 ppg;
const int  HR_WIN = 5;          // moving average window for BPM
float hrBuf[HR_WIN]; int hrIdx=0, hrCount=0;
unsigned long lastBeatMs = 0;

// ================= MLX90614 (Temp) =================
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// ================= ADXL345 (Motion) ================
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
sensors_event_t accel;
bool haveLastAccel=false;
float lastX=0, lastY=0, lastZ=0;
float motionEMA=0.0f;
const float MOTION_ALPHA = 0.20f; // 0..1 (higher = faster)

// ================= Thresholds (tune these) =========
const float TEMP_HYPER_C = 37.5;   // > 37.5°C
const float TEMP_HYPO_C  = 36.0;   // < 36.0°C
const int   HR_HYPER_BPM = 100;    // > 100 bpm
const int   HR_HYPO_BPM  = 60;     // < 60 bpm

const float MOTION_HYPER = 0.20;   // vigorous (tremor/restless)
const float MOTION_HYPO  = 0.05;   // very low activity

const float SWEAT_HIGH_PCT = 15.0; // |ΔGSR%| vs baseline for “sweat high”
const float SWEAT_MED_PCT  = 8.0;  // “sweat mild”

// ================= Helpers =================
void lcdCenter(const String &a, const String &b){
lcd.clear();
lcd.setCursor(max(0, (16-(int)a.length())/2), 0); lcd.print(a.substring(0,16));
lcd.setCursor(max(0, (16-(int)b.length())/2), 1); lcd.print(b.substring(0,16));
}

float meanHR(){
int n = min(hrCount, HR_WIN);
if (n==0) return NAN;
float s=0; for(int i=0;i<n;i++) s+=hrBuf[i];
return s/n;
}
void pushHR(float bpm){
hrBuf[hrIdx]=bpm; hrIdx=(hrIdx+1)%HR_WIN; if(hrCount<HR_WIN) hrCount++;
}

float pushGSRandAvg(int raw){
gsrBuf[gsrIdx]=raw; gsrIdx=(gsrIdx+1)%GSR_SMA; if(gsrCount<GSR_SMA) gsrCount++;
long sum=0; for(int i=0;i<gsrCount;i++) sum+=gsrBuf[i];
return (float)sum/max(1,gsrCount);
}

// ================= Setup =================
void setup(){
Wire.begin(21,22);
Serial.begin(115200);

// LCD
lcd.init(); lcd.backlight();
lcdCenter("Thyroid Proto", "Initializing...");

// MLX90614
if(!mlx.begin()){ Serial.println("MLX90614 not found"); lcdCenter("MLX90614 ERROR","Check wiring"); delay(1500); }

// ADXL345
if(!adxl.begin()){ Serial.println("ADXL345 not found"); lcdCenter("ADXL345 ERROR","Check wiring"); delay(1500); }
else adxl.setRange(ADXL345_RANGE_2_G);

// MAX3010x
if(!ppg.begin(Wire, I2C_SPEED_FAST)){
   Serial.println("MAX3010x not found");
   lcdCenter("MAX3010x ERROR","Check wiring"); delay(1500);
} else {
   byte ledBrightness=0x3F; byte sampleAverage=4; byte ledMode=2;
   int sampleRate=100; int pulseWidth=411; int adcRange=16384;
   ppg.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

// GSR ADC
analogReadResolution(12);
analogSetPinAttenuation(GSR_PIN, ADC_11db);

// GSR baseline
unsigned long t0=millis(); long sum=0; int n=0;
while(millis()-t0<GSR_BASE_MS){ sum+=analogRead(GSR_PIN); n++; delay(50); }
gsrBaseline = (n>0)? (float)sum/n : 2000.0f;
Serial.print("GSR baseline: "); Serial.println(gsrBaseline,1);

lcdCenter("Thyroid Proto","Ready");
delay(800);
}

// ================= Loop =================
unsigned long lastLCD=0;

void loop(){
// -------- Heart Rate (PPG) --------
long ir = ppg.getIR();
if(ir>5000 && checkForBeat(ir)){
   unsigned long now=millis();
   unsigned long delta=now-lastBeatMs; lastBeatMs=now;
   if(delta>300 && delta<2000){  // 30..200 bpm
     float bpm=60.0f*1000.0f/(float)delta;
     pushHR(bpm);
   }
}
float bpmAvg = meanHR();
bool hrValid = !isnan(bpmAvg) && bpmAvg>30 && bpmAvg<200;

// -------- Temperature --------
float tempC = mlx.readObjectTempC(); // NAN if sensor not ready sometimes

// -------- Motion (ADXL) --------
adxl.getEvent(&accel);
float motion = NAN;
if(haveLastAccel){
   float dx=accel.acceleration.x-lastX;
   float dy=accel.acceleration.y-lastY;
   float dz=accel.acceleration.z-lastZ;
   float inst = sqrt(dx*dx+dy*dy+dz*dz);
   motionEMA = MOTION_ALPHA*inst + (1.0f-MOTION_ALPHA)*motionEMA;
   motion = motionEMA;
}
lastX=accel.acceleration.x; lastY=accel.acceleration.y; lastZ=accel.acceleration.z; haveLastAccel=true;

// -------- GSR (sweat) --------
int gsrRaw = analogRead(GSR_PIN);
float gsrAvg = pushGSRandAvg(gsrRaw);
float gsrPct = (gsrBaseline>1.0f)? ((gsrAvg-gsrBaseline)*100.0f/gsrBaseline) : 0.0f;
String sweatLevel = "DRY";
if (fabs(gsrPct) > SWEAT_HIGH_PCT)      sweatLevel = "HIGH";
else if (fabs(gsrPct) > SWEAT_MED_PCT)  sweatLevel = "MILD";

// -------- Screening Logic (prototype) --------
String status = "NORMAL";
if (!hrValid || isnan(tempC)) {
   status = "PLACE FINGER";
} else {
   bool hyper = (bpmAvg > HR_HYPER_BPM) && (tempC > TEMP_HYPER_C) &&
                ( (!isnan(motion) && motion > MOTION_HYPER) || (sweatLevel=="HIGH") );
   bool hypo  = (bpmAvg < HR_HYPO_BPM) && (tempC < TEMP_HYPO_C) &&
                ( (!isnan(motion) && motion < MOTION_HYPO) );

   if (hyper)      status = "POSSIBLE HYPER";
   else if (hypo)  status = "POSSIBLE HYPO";
   else            status = "NORMAL";
}

// -------- LCD update (4x/sec) --------
if(millis()-lastLCD>250){
   lastLCD=millis();
   lcd.clear();
   // L1: HR + Temp
   lcd.setCursor(0,0);
   if(hrValid){ lcd.print("HR:"); lcd.print((int)round(bpmAvg)); lcd.print(" "); }
   else        { lcd.print("HR:-- "); }
   lcd.print("T:");
   if(!isnan(tempC)) { lcd.print(tempC,1); }
   else              { lcd.print("--.-"); }

   // L2: Motion + Sweat + Status short
   lcd.setCursor(0,1);
   lcd.print("M:");
   if(!isnan(motion)) lcd.print(motion,2); else lcd.print("--");
   lcd.print(" S:");
   lcd.print(sweatLevel.substring(0,1));  // D / M / H
   lcd.print(" ");

   if(status=="POSSIBLE HYPER") lcd.print("HYPER");
   else if(status=="POSSIBLE HYPO") lcd.print("HYPO ");
   else if(status=="PLACE FINGER") lcd.print("FINGER");
   else lcd.print("NORMAL");
}

// Optional serial debug (slower)
static unsigned long lastDbg=0;
if(millis()-lastDbg>1000){
   lastDbg=millis();
   Serial.print("HR="); Serial.print(hrValid? bpmAvg : -1);
   Serial.print("  T="); Serial.print(tempC);
   Serial.print("  M="); Serial.print(motion);
   Serial.print("  GSRraw="); Serial.print(gsrRaw);
   Serial.print("  dGSR%="); Serial.print(gsrPct,1);
   Serial.print("  Sweat="); Serial.print(sweatLevel);
   Serial.print("  -> "); Serial.println(status);
}
}
