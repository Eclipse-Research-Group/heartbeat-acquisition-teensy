#include <HardwareSerial.h>
#include <Adafruit_GPS.h>
#include <TimeLib.h>
// #include <TeensyTimerTool.h>

// #define DEBUG
// #define _BOARD_VER_2
// #define _BOARD_VER_1_2
#define BOARD_VER_1_2

#ifdef BOARD_VER_1_2
#define PIN_GPS_PPS 2
#define PIN_GPS_FIX 5
#define PIN_LED_GPSFIX 3
#define PIN_LED_XFER 6
#define PIN_LED_OVRLD 4
#define PIN_ANALOG_IN 0
#endif

#ifdef BOARD_VER_1_0
#define PIN_GPS_PPS 2
#define PIN_GPS_FIX 5
#define PIN_LED_GPSFIX 3
#define PIN_LED_OVRLD 4
#define PIN_ANALOG_IN 0
#endif


#define SAMPLE_RATE 20000
#define CAPTURE_TIME_MS 360
#define SAMPLE_SIZE CAPTURE_TIME_MS* SAMPLE_RATE / 1000
#define SAMPLE_TIME_US (1000000 / SAMPLE_RATE)
#define CLIP_THRESHOLD 10


// ADC stuff
IntervalTimer ADCTimer;
IntervalTimer secondTimer;
IntervalTimer checkTImer;

bool second_timer_running = false;
volatile unsigned int data[SAMPLE_SIZE];
volatile unsigned int sample_count = 0;
volatile bool data_ready = false;
bool pps_start = false;

// GPS serial port
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
bool GPS_fix = false;

// Serial port to raspberry pi
#define AcqSerial Serial1

// Capture variables
bool capture_did_clip = false;

bool toggle = false;
int pps_count = 0;

void ISR_GPS_pps() {
  pps_count += 1;
  start_capture();
}

void ISR_GPS_fix() {
  if (pps_count > 0 && pps_count < 18) {
    digitalWrite(PIN_LED_GPSFIX, HIGH);
    GPS_fix = true;
  } else {
    digitalWrite(PIN_LED_GPSFIX, LOW);
    GPS_fix = false;
  }

  pps_count = 0;
}

void DoADC() {
  noInterrupts();
  int analog_value = 0;

  if (sample_count < SAMPLE_SIZE) {
    analog_value = analogRead(PIN_ANALOG_IN);

    if ((analog_value > 1023 - CLIP_THRESHOLD) || analog_value < CLIP_THRESHOLD) {
      // we're close to clipping
      capture_did_clip = true;
      digitalWrite(PIN_LED_OVRLD, HIGH);
    }

    data[sample_count] = analog_value;
    sample_count++;
  } else {
    ADCTimer.end();
    digitalWrite(LED_BUILTIN, LOW);
    data_ready = true;
  }
  interrupts();
}

void ISR_Second_tick() {
  start_capture();
}

void start_capture() {

  // Reset clipping
  capture_did_clip = false;
  digitalWrite(PIN_LED_OVRLD, LOW);

  // Call ADC once before timer
  DoADC();

  // Start ADC timer
  ADCTimer.begin(DoADC, SAMPLE_TIME_US);

  // PPS start
  pps_start = true;
  digitalWrite(LED_BUILTIN, HIGH);
}

time_t get_unix_time(int year, int month, int day, int hour, int minute, int second) {
  tmElements_t tm;
  tm.Year = year - 1970;
  tm.Month = month;
  tm.Day = day;
  tm.Hour = hour;
  tm.Minute = minute;
  tm.Second = second;
  return makeTime(tm);
}

void setup() {
  AcqSerial.begin(9600);
  AcqSerial.println("# Welcome to the ECLIPSE DATA ACQUISITION BOARD...");
#ifdef BOARD_VER_1_0
  AcqSerial.println("# Board version: 1.0");
#endif

#ifdef BOARD_VER_1_2
  AcqSerial.println("# Board version: 1.2");
#endif
  AcqSerial.printf("# ADC sampling at %d Hz (%d us per sample).\n", SAMPLE_RATE, SAMPLE_TIME_US);
  AcqSerial.printf("# %d time, %d sample size\n", CAPTURE_TIME_MS, SAMPLE_SIZE);


  // Configure interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), ISR_GPS_pps, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_FIX), ISR_GPS_fix, RISING);

  // Configure pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED_OVRLD, OUTPUT);
  pinMode(PIN_LED_GPSFIX, OUTPUT);

#ifdef BOARD_VER_1_2
  pinMode(PIN_LED_XFER, OUTPUT);
#endif

  pinMode(PIN_GPS_FIX, INPUT);
  pinMode(PIN_GPS_PPS, INPUT);

  // Default pin state
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(PIN_LED_GPSFIX, LOW);
  digitalWrite(PIN_LED_OVRLD, LOW);
#ifdef BOARD_VER_1_2
  digitalWrite(PIN_LED_XFER, LOW);
#endif

  GPSSerial.begin(9600);
  AcqSerial.begin(1000000);
}

time_t lastTime = 0;

void loop() {
  GPS.read();

  unsigned int checksum = 0;

  if (!GPS_fix && !second_timer_running) {
    second_timer_running = true;
    secondTimer.begin(ISR_Second_tick, 1e6);
  } else if (GPS_fix && second_timer_running) {
    noInterrupts();
    secondTimer.end();
    second_timer_running = false;
    interrupts();
  }

  if (GPS.newNMEAreceived()) {
#ifdef DEBUG
    AcqSerial.println("# Recieved GPS NMEA message");
    AcqSerial.print("# ");
    AcqSerial.print(GPS.lastNMEA());
#endif

    GPS.parse(GPS.lastNMEA());
  }

  time_t last_time = 0;

  if (data_ready) {
    time_t time = get_unix_time(2000 + GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);

    if (last_time == time) {
      AcqSerial.println("#! GPS time hasn't been updated since last tick!");
      time = -1;
    }


    // For whatever reason the time value is incorrect on the first tick when device is reset. This is a hacky fix.
    if (time < 1705819583) {
      AcqSerial.println("#! Time is too early, out of sync");
      time = -1;
    }

#ifdef BOARD_VER_1_2
    digitalWrite(PIN_LED_XFER, HIGH);
#endif

    AcqSerial.print("$");

    // Time
    AcqSerial.print(time);
    AcqSerial.print(",");

    // Flags
    if (GPS_fix) {
      AcqSerial.print("G");
    }

    if (capture_did_clip) {
      AcqSerial.print("O");
    }
    AcqSerial.print(",");

    // Sample rate
    AcqSerial.print(SAMPLE_RATE);
    AcqSerial.print(",");

    // Print GPS
    AcqSerial.printf("%f,%f,%f,", GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.altitude);

    // Print satellite count
    AcqSerial.printf("%u,", GPS.satellites);

    // Print speed and angle
    AcqSerial.printf("%f,%f,", GPS.speed, GPS.angle);

    // Print data point count
    AcqSerial.printf("%u,", sample_count);

    for (unsigned int i = 0; i < sample_count; i++) {
      AcqSerial.print(data[i]);
      checksum += data[i];
      if (i < sample_count - 1) {
        AcqSerial.print(",");
      }
    }

    AcqSerial.printf(",%u", checksum);


    AcqSerial.println();

    // acq_error:

#ifdef BOARD_VER_1_2
    digitalWrite(PIN_LED_XFER, LOW);
#endif

    data_ready = false;
    sample_count = 0;

    if (time != -1) {
      last_time = time;
    }

    // setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year + 2000);
  }
}
