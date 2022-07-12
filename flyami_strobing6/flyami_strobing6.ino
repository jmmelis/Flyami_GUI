#include <ros.h>
#include <std_msgs/String.h>

// pins:
int wb_signal_pin = 14;
int wb_thresh_pin = A1;
int wb_phase_pin = A2;
int blue_LED_pin = 3;
int hs_sync_pin = 4;
int cam_strobe_pin = 5;
int trigger_in_pin = 17;
int opto_pin_in = 10;
int opto_pin_out = 9;
const int N_trig_pin = 4;
const int trig_out_pins[N_trig_pin] = {18,19,20,8};

unsigned long t_count = 0;
unsigned long t_high = 0;
unsigned long t_low = 0;
unsigned long t_last = 0;
unsigned long t_strobe_high = 0;
unsigned long t_strobe_low = 0;
unsigned long t_opto_high = 0;
unsigned long t_opto_low = 0;
unsigned long t_last_pulse = 0;

int t_break = 150;
int t_wb_min = 50;
int t_wb_max = 130;
int t_strobe = 75;

int LED_pulse = 20;
int OPTO_pulse = 5; // Duration opto-pulse in counts
float OPTO_phase = 0.5; // Phase of opto strobing in the wingbeat

int wb_duration = 0;
int wb_buffer_ind = 0;
const int wb_buffer_size = 25;
float wb_buffer[wb_buffer_size];
int strobe_period = 75;

int cam_strobe_counter = 0;

float thresh_val;
float phase_val;

bool fly_flying = false;
bool cam_on = false;
bool opto_pulse = false;

int OPTO_pulse_count = 0;

/*
int pulse_wb[100] = {1,0,0,0,0,0,0,0,0,0, \
                    0,1,1,0,0,0,0,0,0,0, \
                    0,0,0,1,1,1,1,1,0,0, \
                    0,0,0,0,0,0,0,0,1,1, \
                    1,1,1,1,1,1,1,1,0,0, \
                    0,0,0,0,0,0,0,0,1,1, \
                    1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,0,0, \
                    0,0,0,0,0,0,0,0,0,0, \
                    0,0,0,0,0,0,0,0,0,0};
*/


int pulse_wb[100] = {1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,1,1, \
                    1,1,1,1,1,1,1,1,1,1};
                    

bool OPTO_wb = false;

// ROS node:
ros::NodeHandle nh; // publisher

std_msgs::String frame_msg;

ros::Publisher frame_nr("frame_nr", &frame_msg);

void setup() {
  
  nh.initNode();
  nh.advertise(frame_nr);

  delay(100);
  
  // setup pins:
  pinMode(wb_signal_pin, INPUT);
  pinMode(wb_thresh_pin, INPUT);
  pinMode(wb_phase_pin, INPUT);
  pinMode(hs_sync_pin, INPUT);
  pinMode(blue_LED_pin, OUTPUT);
  pinMode(cam_strobe_pin, OUTPUT);
  pinMode(trigger_in_pin, INPUT);
  pinMode(opto_pin_in,INPUT);
  pinMode(opto_pin_out, OUTPUT);
  digitalWriteFast(opto_pin_out,LOW);
  for (int n=0; n<N_trig_pin; n++) {
    pinMode(trig_out_pins[n], OUTPUT);
  }

  delay(1000);

  attachInterrupt(hs_sync_pin,hs_sync,RISING);

  delay(100);

  attachInterrupt(wb_signal_pin,wb_sync,FALLING);

  delay(100);

  attachInterrupt(trigger_in_pin,fire_trigger,RISING);
}

void loop() {
  
  // Refresh ros nodes:
  nh.spinOnce();
  
  // Check if fly is flying:
  if (wb_duration>t_wb_min && wb_duration<t_wb_max && t_count<(t_last+t_break)) {
    fly_flying = true;
  }
  else {
    fly_flying = false;
  }

  // Check if opto pin is on:
  if (digitalReadFast(opto_pin_in)==HIGH) {
    opto_pulse = true;
  }
  else {
    opto_pulse = false;
    OPTO_pulse_count = 0;
  }

  // Check phase value:
  thresh_val = analogRead(wb_thresh_pin)/1024.0;
  //phase_val = analogRead(wb_phase_pin)/1024.0;
  phase_val = -0.75+analogRead(wb_phase_pin)/1024.0;

  delay(2);
}

void frame_nr_publish(uint32_t frame_nr_in, int mode) {
  if (mode == 0) {
    char buffer[12];
    snprintf(buffer, 12, "F%010lu\n", frame_nr_in);   
    frame_msg.data = buffer;
  }
  else if (mode == 1) {
    char buffer[12];
    snprintf(buffer, 12, "T%010lu\n", frame_nr_in); 
    frame_msg.data = buffer;
  }
  else if (mode == 2) {
    char buffer[12];
    snprintf(buffer, 12, "L%010lu\n", frame_nr_in); 
    frame_msg.data = buffer;
  }
  else if (mode == 3) {
    char buffer[12];
    snprintf(buffer, 12, "N%010lu\n", frame_nr_in); 
    frame_msg.data = buffer;
  }
  frame_nr.publish( &frame_msg );
  nh.spinOnce();
}

void wb_sync() {
  // Calculate wingbeat duration:
  wb_buffer[wb_buffer_ind%wb_buffer_size] = t_count-t_last;
  wb_buffer_ind++;
  t_last = t_count;

  float wb_t = 0.0;
  for (int i=0; i<wb_buffer_size; i++) {
    wb_t += wb_buffer[i]/(wb_buffer_size*1.0);
  }
  wb_duration = (int) round(wb_t);

  // Calculate strobe pulses:
  if (fly_flying) {
    t_high = (int) t_count+round(1.0*wb_duration*phase_val);
    t_low = (int) t_count+round(1.0*wb_duration*phase_val)+LED_pulse;
  }

  // Calculate opto pulses:
  if (opto_pulse==true && pulse_wb[OPTO_pulse_count%100]==1) {
    t_opto_high = t_count;
    t_opto_low = t_count+wb_duration;
  }
  OPTO_pulse_count++;
}

void hs_sync() {
  noInterrupts();
  t_count++;
  interrupts();

  if (t_count > t_opto_high && t_opto_high>0) {
    digitalWriteFast(opto_pin_out,HIGH);
    frame_nr_publish(t_count,2);
    t_opto_high= 0;
  }
  else if (t_count > t_opto_low && t_opto_low>0) {
    digitalWriteFast(opto_pin_out,LOW);
    frame_nr_publish(t_count,2);
    t_opto_low=0;
  }

  if (fly_flying==false) {
    if (t_count%strobe_period==0) {
      digitalWriteFast(blue_LED_pin,LOW);
      if (cam_strobe_counter%2==0) {
        digitalWriteFast(cam_strobe_pin,LOW);
        frame_nr_publish(t_count,3);
      }
    }
    else if ((t_count+LED_pulse)%strobe_period==0) {
      digitalWriteFast(blue_LED_pin,HIGH);
      if (cam_strobe_counter%2==0) {
        digitalWriteFast(cam_strobe_pin,HIGH);
      }
      cam_strobe_counter++;
    }
  }
  else {
    
    // Blue LED output:
    if (t_count > t_high && t_high>0) {
      digitalWriteFast(blue_LED_pin,HIGH);
      if (cam_on==false && t_count>(t_last_pulse+30)) {
        digitalWriteFast(cam_strobe_pin,HIGH);
        cam_on = true;
        t_strobe_low = t_count+wb_duration;
      }
      t_high=0;
    }
    else if (t_count > t_low && t_low>0) {
      digitalWriteFast(blue_LED_pin,LOW);
      if (cam_on==true && t_strobe_low<t_count) {
        digitalWriteFast(cam_strobe_pin,LOW);
        frame_nr_publish(t_count,0);
        cam_on = false;
        t_last_pulse = t_count;
      }
      t_low = 0;
    }
  }
}

void fire_trigger() {
  noInterrupts();
  for (int n=0; n<N_trig_pin; n++) {
    digitalWriteFast(trig_out_pins[n],HIGH);
  }
  delayMicroseconds(10);
  for (int n=0; n<N_trig_pin; n++) {
    digitalWriteFast(trig_out_pins[n],LOW);
  }
  frame_nr_publish(t_count,1);
  interrupts();
}
