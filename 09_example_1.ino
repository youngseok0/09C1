// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.7    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

#define N 30              // length to store previous values

// global variables
unsigned long last_sampling_time;   // unit: msec
float dist_prev = _DIST_MAX;        // Distance last-measured
float dist_ema, dist_median;        // EMA distance

float previous_samples[N];          // queue that store preivous N samples
float temp_array[N];                // queue that copy previous_samples to modify queue without changing original one

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // initialize serial port
  Serial.begin(57600);

  // initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
  float dist_raw;
  
  // wait until next sampling time. 
  // millis() returns the number of milliseconds since the program started. 
  // Will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  
  if (dist_raw < _DIST_MIN) {
    dist_raw =  _DIST_MIN - 10.0;    // Set Lower Value
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw > _DIST_MAX) {
    dist_raw =  _DIST_MAX + 10.0;    // Set Higher Value
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // In desired Range
    digitalWrite(PIN_LED, 0);       // LED ON      
  }
  
  // Modify the below line to implement the EMA equation
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  enqueue(previous_samples, dist_raw); // enqueue current value
  dist_median = getMedian(previous_samples); // get median value

  // output the distance to the serial port
  Serial.print("Min:");   Serial.print(_DIST_MIN);
  Serial.print(",raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");  Serial.print(dist_ema);
  Serial.print(",median:");  Serial.print(dist_median);
  Serial.print(",Max:");  Serial.print(_DIST_MAX);
  Serial.println("");

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm

  // Pulse duration to distance conversion example (target distance = 17.3m)
  // - round trip distance: 34.6m
  // - expected pulse duration: 0.1 sec, or 100,000us
  // - pulseIn(ECHO, HIGH, timeout) * 0.001 * 0.5 * SND_VEL
  //        = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
  //        = 100,000 * 0.001 * 0.5 * 346 * micro * sec * milli * meter
  //                                        ----------------------------
  //                                         micro * sec
  //        = 100 * 173 milli*meter = 17,300 mm = 17.3m
  // pulseIn() returns microseconds.
}

// append current value to queue
void enqueue(float *queue, float curr_val) {
  for(int i = N; i > 0; i--) {
    queue[i] = queue[i-1];
  }
  queue[0] = curr_val;
}

// sort array acending
void sort(float *queue) {
  float min_val, temp;
  int index;
  
  for(int i=0; i< N; i++){
    min_val = 100000;
    for(int j = i; j < N; j++){
      if(min_val > queue[j]) {
        min_val = queue[j];
        index = j;
      }
    }
    temp = queue[i];
    queue[i] = queue[index];
    queue[index] = temp;
  }
}

// copy array source array to destination array
// ** Arrays must have length N **
void copyArray(float *dst_array, float *src_array) {
  for(int i=0; i<N; i++) {
    dst_array[i] = src_array[i];
  }
}

// return median value from array length N 
float getMedian(float *queue) {
  copyArray(temp_array, queue);
  sort(temp_array);
  
  return temp_array[(int)N/2];
}
