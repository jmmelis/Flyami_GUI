int hs_pin = 0;
int opto_in_pin = 1;
int switch_1_pin = 2;
int switch_2_pin = 3;
int opto_out_pin = 4;
int LED_pin = 13;

bool calib_on = true;
bool pulse_on = false;

unsigned long count = 0;

int duty_cycle = 10;

void setup() {

  pinMode(hs_pin,INPUT);
  pinMode(opto_in_pin,INPUT);
  pinMode(switch_1_pin,INPUT);
  pinMode(switch_2_pin,INPUT);
  pinMode(opto_out_pin,OUTPUT);
  pinMode(LED_pin, OUTPUT);

  delay(1000);

  attachInterrupt(hs_pin,hs_sync,RISING);
}

void loop() {
  
  if (digitalRead(switch_1_pin)==HIGH) {
    // calibration phase, low level laser light
    duty_cycle = 100;
    calib_on = true;
  }

  if (digitalRead(switch_2_pin)==HIGH) {
    // activation phase, high level laser light
    duty_cycle = 2;
    calib_on = false;
  }

  if (digitalRead(opto_in_pin)==HIGH) {
    // strobe laser:
    pulse_on = true;
  }
  else {
    pulse_on = false;
  }

  delay(10);
}

void hs_sync() {
  //noInterrupts();
  count++;
  if (pulse_on==true || calib_on==true) {
    if (count%duty_cycle==0) {
      digitalWriteFast(opto_out_pin,HIGH);
      delayMicroseconds(20);
      digitalWriteFast(opto_out_pin,LOW);
    }
    else {
      digitalWriteFast(opto_out_pin,LOW);
    }
  }
  //interrupts();
}
