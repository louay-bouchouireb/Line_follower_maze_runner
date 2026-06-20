 int qtr[8] = {14, 27, 26, 25, 33, 32, 35, 34};
  int sensor_min[8] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
  int sensor_max[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int weight[8] = {-350, -250, -150, -50, 50, 150, 250, 350};
  float last_position = 0;
  float previous_error = 0;
  float integral = 0;
  unsigned long victory_start_time = 0;
  float kp = 2.7 , kd = 51.0 , ki = 0.0;

  int IN1 = 4;
  int IN2 = 16;
  int IN3 = 17;
  int IN4 = 5;
  int ENA = 18;
  int ENB = 19;
int base_speed = 90; 
int y, z, t;
float pid(float error);
void reading_sensor(int* reading);
float Line_Position(int* reading);
int mapping(int mini, int maxi, int reading);
float integral_limit = 10000;
bool line_detected = false;
bool was_line_detected = false;
void set_motors(float correction);
void stopm();
void turn_back();
 


void setup() {
  // Callibration led

  pinMode(2,OUTPUT);
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);
  digitalWrite(2, HIGH);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  int x;
  int counter = 5000;

while(counter > 0){
  for (int i = 0; i<8;i++){
      x = analogRead(qtr[i]);
      if (x < sensor_min[i])
        sensor_min[i] = x;
      if (x > sensor_max[i])
        sensor_max[i] = x;
    }
    delay (1);
    counter --;
}
// Turn off callibration led
 digitalWrite(2, LOW);

}

void loop() {
  int read[8];
  int prevread[8];
  float correction;
  float pos = 0;
  int s = 0;
  reading_sensor(read);
  // --- VICTORY SQUARE DETECTION ---
  bool all_black = true;
  for(int i = 0; i < 7; i++){
    if(read[i] < 400) { // If any sensor sees white, reset
      all_black = false;
      break;
    }
  }

  if(all_black) {
    if(victory_start_time == 0) {
           // stopm();

      victory_start_time = millis(); 
    }
    else if (millis() - victory_start_time > 100) { 
      stopm();
      while(1);
    }
  } else {
    victory_start_time = 0; 
  }
  pos = Line_Position(read);
  correction = pid(pos);
  

if (read [0] < 300 && read [1] < 300 && read [2] < 300 && read [3] < 300 &&  read [6] < 300 && read [7] < 300 && read [5] < 300 && read [4] < 300){
turn_back();
  reading_sensor(read);}

  if (read[0] > 700  && read [7] > 700 && read[1] > 700 && read[6] > 700 ){
  
    
    stopm();
 
    read[0] = 0;
    read[1] = 0;
    read[2] = 0;



  pos = Line_Position(read);
  correction = pid(pos);
  }
 
  


 
  set_motors(correction);


  

}


int mapping(int mini, int maxi, int reading){

if (maxi == mini){return 0;}
else
return ((reading - mini)*1000L/(maxi-mini));

}





void reading_sensor(int* reading){
  int k = 0;
  for (int i = 0; i<8; i++){
   k = mapping(sensor_min[i],sensor_max[i], analogRead(qtr[i]));
   if (k < 0) k = 0;
   if (k > 1000) k = 1000;
   if (k < 100){
    k = 0;
   }
   reading[i] = k;

  }
}



float Line_Position(int *reading){

  long sum1 = 0;
  int sum2 = 0;
  for (int i = 0; i<8; i++){
  sum1 = sum1 + (long)reading[i]*weight[i];
  sum2 = sum2 + reading[i];
  }
  if (sum2 == 0){
    line_detected = false;
    return last_position;
  }
  else{
    line_detected = true;
    last_position = (float)sum1/sum2;
  return (last_position);
  }
}

float pid(float error){
 if (line_detected) {
    integral = integral + error;
  } else {
    integral = 0; 
  }

  if (integral > integral_limit) integral = integral_limit;
  if (integral < -integral_limit) integral = -integral_limit;
  
  float p, i_term, d;
  

  p = kp * error;
  i_term = ki * integral;


  if (line_detected && !was_line_detected){
    previous_error = error;
  }
  d = kd *(error - previous_error);
  previous_error = error;
  was_line_detected = line_detected;

return (p + i_term + d);
}


void set_motors(float correction) {
  int left_speed = base_speed + correction;
  int right_speed = base_speed - correction;

  if (left_speed > 255) left_speed = 255;
  if (left_speed < -255) left_speed = -255;
  if (right_speed > 255) right_speed = 255;
  if (right_speed < -255) right_speed = -255;

  if (left_speed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, left_speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -left_speed);
  }

  if (right_speed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, right_speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -right_speed);
  }
}

void stopm(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}


void turn_back() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, 50);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, 50);

  delay(400);

  int r[8];
  unsigned long start = millis();
  while (millis() - start < 3000) {
    reading_sensor(r);
    if (r[3] > 500 || r[4] > 500) break;
  }
  stopm();
}
