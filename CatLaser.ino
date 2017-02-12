#include <Accel.h>
#include <Quaternion.h>
#include <Servo.h> 
#include <FlashStorage.h>

#define ACCEL_INTERVAL_MS 10 // 100 samples per second
#define NUM_TRACK_POINTS 1024

#define GND_PIN_9DOF 5
#define _3V_PIN_9DOF 6
#define VIN_PIN_9DOF 9
#define PITCH_PIN 10
#define YAW_PIN 11
#define BUTTON_PIN 12
#define LED_PIN 13

typedef struct {
   float x;
   float y;
   float z;
} AbsolutePoint; // 12 bytes

typedef struct {
   AbsolutePoint points[NUM_TRACK_POINTS]; // 12KB
   int n;
} Track;

Accel accel(ACCEL_INTERVAL_MS);
FlashStorage(trackStore, Track);
Track track;

Servo pitch;
Servo yaw;

void setup() {
  Serial.begin(115200);

  pinMode(GND_PIN_9DOF, OUTPUT);
  digitalWrite(GND_PIN_9DOF, LOW);
  pinMode(_3V_PIN_9DOF, OUTPUT);
  digitalWrite(_3V_PIN_9DOF, HIGH);
  pinMode(VIN_PIN_9DOF, OUTPUT);
  digitalWrite(VIN_PIN_9DOF, HIGH);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);

  pitch.attach(PITCH_PIN);
  pitch.write(90);
  yaw.attach(YAW_PIN);
  yaw.write(90);
  
  delay(1000);
  trackStore.read(&track);
  accel.begin();
}


int currentTrackPoint = 0;

void loop() {
  bool didUpdate = accel.Update();
  if (updateTracks(didUpdate)) {
    return;
  }
  
  if (!didUpdate) {
    return;
  }
    Quaternion absolutePosition(1, 0, 0);
    if (track.n > 0) {
      AbsolutePoint p = track.points[currentTrackPoint];
      currentTrackPoint = (currentTrackPoint + 1) % track.n;
      absolutePosition = Quaternion(p.x, p.y, p.z);
    }
    
    Quaternion q = accel.getDeviceOrientation(absolutePosition);
    float phi = acos(q.d) * -180.0f / PI + 180; // acos is from 0 to pi
    float theta = atan2(q.c, q.b) * 180.0f / PI + 90; // atan2 is from -pi to pi
  
    if (theta < 0) {
      theta = 0;
    } else if (theta > 180) {
      theta = 180;
    }
    
    pitch.write(phi);
    yaw.write(theta);
    //Serial.print(q.b); Serial.print(" "); Serial.print(q.c); Serial.print(" "); Serial.println(q.d); 
}

bool wasButtonDown = false;
int buttonDownStart = 0;

// This method checks if the track update button is pressed and updates the tracks.
// This method return true if track recording is happening.
bool updateTracks(bool didUpdate) {
  int buttonRead = digitalRead(BUTTON_PIN);
  if (LOW == buttonRead) {
    int currentMillis = millis();
    if (!wasButtonDown) {
      wasButtonDown = true;
      buttonDownStart = currentMillis;
      yaw.write(90);
      pitch.write(90);
      track.n = 0;
      return true;
    } else if (currentMillis - buttonDownStart < 1000) {
      // wait 1 second before tracking for the servos to move
      return true;
    } else if (NUM_TRACK_POINTS == track.n) {
      // we can't add any more points because we are full up
      digitalWrite(LED_PIN, LOW);
      return true;
    }

    digitalWrite(LED_PIN, HIGH);  // turn on led to show we are tracking
    if (didUpdate) {
      Quaternion q = accel.getAbsoluteOrientation(Quaternion(1, 0, 0));
      track.points[track.n].x = q.b;
      track.points[track.n].y = q.c;
      track.points[track.n].z = q.d;
      track.n++;
    }
    return true;
  }
  
  if (wasButtonDown) {
    // LED off, we are done tracking
    digitalWrite(LED_PIN, LOW);
    wasButtonDown = false;
    currentTrackPoint = 0;
    if (track.n > 0) {
      trackStore.write(track);
    } else {
      // Restore old track data because we didn't make a new track.
      trackStore.read(&track);
    }
  }
  return false;
}

