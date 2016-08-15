byte PWM_PIN = 3;
 
int pwm_value;
 
void setup() {
  pinMode(PWM_PIN, INPUT);
  Serial.begin(115200);
}
 
void loop() {
  pwm_value = pulseIn(PWM_PIN, HIGH);
  Serial.println(pwm_value);
}
