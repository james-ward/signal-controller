#include <Wire.h>
#define ENABLE_DEBUG_OUTPUT
#include <Adafruit_PWMServoDriver.h>

TwoWire i2c;

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40, i2c);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41, i2c);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42, i2c);
#define SERVOMIN  87 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  470 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

bool sequence_running = false;
unsigned long heartbeat = millis();

class Signal;

enum class BoardColour {
  YELLOW,
  ROSE,  // PINK is used in a macro for the MEGA2560
  BLUE,
};

void setPosition(int servonum, Adafruit_PWMServoDriver& pwm, float pos_degrees) {
  float pulselen = map(pos_degrees, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servonum, 0, pulselen);
}

enum class State {
  INIT,
  CLEAR,
  DANGER,
  MOVING_CLEAR,
  MOVING_DANGER,
};

class Signal {
public:
  Signal(const unsigned char servo_number, Adafruit_PWMServoDriver& pwm, const unsigned char clear_degrees, const unsigned char danger_degrees) :
  servo(servo_number), pwm(pwm), clear_deg(clear_degrees), danger_deg(danger_degrees) {}
  void update() {
    static int startup_delay = 0;
    auto elapsed = millis() - t;
    float pos;
    switch (state) {
      case State::INIT:
        set_degrees(0.);  // DANGER by default
        startup_delay++;
        if (startup_delay >= 300) {
          state = State::DANGER;
        }
        break;
      case State::MOVING_CLEAR:
        if (elapsed < up_time_ms / 2) {
          pos = (-cos(elapsed / (up_time_ms/2.) * 3.14159) + 1.0) / 4.0;
        } else {
          pos = 0.5 + (-cos((elapsed - up_time_ms/2.) / (up_time_ms/2.) * 3.14159) + 1.0) / 4.0;
        }
        if (elapsed > up_time_ms) {
          state = State::CLEAR;
        }
        set_degrees(pos);
        break;
      case State::MOVING_DANGER:
        if (elapsed < 0.70 * down_time_ms) {
          pos = (cos(elapsed / (0.70 * down_time_ms) * 3.14159/2.0));
        } else {
          pos = (1.0-cos((elapsed - 0.70 * down_time_ms) / (0.30 * down_time_ms) * 3.14159 * 2.0)) / 2.0 * 0.25;
        }
        if (elapsed > down_time_ms) {
          state = State::DANGER;
        }
        set_degrees(pos);
        break;
      case State::CLEAR:
      case State::DANGER:
        // Don't send each time if we haven't changed
        break;
    }

  }
  void move_clear() {
    if (state == State::DANGER) {
      state = State::MOVING_CLEAR;
      t = millis();
    }
  }
  void move_danger() {
    if (state == State::CLEAR) {
      state = State::MOVING_DANGER;
      t = millis();
    }
  }
  State get_state() {
    return state;
  }
  Adafruit_PWMServoDriver& pwm;
  unsigned char servo;
private:
  unsigned long t = 0;
  State state = State::INIT;
  const unsigned int up_time_ms = 2000;
  const unsigned int down_time_ms = 750;
  const unsigned char clear_deg;
  const unsigned char danger_deg;

  void set_degrees(float pos) {
    static float last_deg = 99.0;
    if (pos > 1.0) {
      pos = 1.0;
    }
    if (pos < 0.0) {
      pos = 0.0;
    }
    float deg = (1.0* clear_deg - 1.0* danger_deg) * pos + 1.0 * danger_deg;
    if(fabs(deg - last_deg) > 0.5) {
      setPosition(servo, pwm, deg);
      last_deg = deg;
    }
  }
};

class SequenceElement {
public:
  Signal* signal;
  State signal_state;
  int time_delay_ms = 1000;

  SequenceElement(Signal* signal, State signal_state, int time_delay_ms_ = 1000): signal(signal), signal_state(signal_state), time_delay_ms(time_delay_ms_) {}
};


const Signal signals[] = {
  // servo number, controller, clear, danger
  // servo degrees are +ve -> ccw
  // yellow
  {1, pwm0, 130, 85}, //0
  {2, pwm0, 80, 115},  
  {3, pwm0, 140, 80},
  {4, pwm0, 40, 95},
  {5, pwm0, 55, 105},
  {6, pwm0, 30, 70}, //- //5
  // ROSE
  {1, pwm1, 95, 55}, //+
  {2, pwm1, 105, 65}, //+
  {3, pwm1, 140, 60}, //+
  {4, pwm1, 80, 130}, //-
  {5, pwm1, 30, 70}, //- //10
  {6, pwm1, 45, 100}, //- 
  {7, pwm1, 60, 105}, //-
  {8, pwm1, 135, 90}, //+
  // blue
  {1, pwm2, 25, 70}, //-
  {2, pwm2, 135, 80}, //+
  {3, pwm2, 140, 85},  //+ //15
  {4, pwm2, 145, 95}, //+
  {5, pwm2, 70, 125}, //-
  {6, pwm2, 95, 60},  //+
  {7, pwm2, 83, 119}, //- //19
};


class Sequence {
public:
  Sequence(SequenceElement* elements, unsigned char size, int start_delay_ms_ = 0) : size{size},
    elements{elements}, start_delay_ms{start_delay_ms_} {}
  void stop() {
    current = nullptr;
    sequence_running = false;
    is_waiting = false;
    t = 0;
  }
  void enqueue() {
    if (current == nullptr) {
      is_waiting = true;
    }
  }
  void update() {
    if (is_waiting && !sequence_running) {
      // Find the first element that is in the wrong state
      current = elements;
      while (current->signal->get_state() == current->signal_state) {
        if (current-elements < size - 1) {
          current++;
        } else {
          stop();
          return;
        }
      }
      is_waiting = false;
      sequence_running = true;
      t = 0;
      start_time = millis();
    }
    if (current == nullptr) {
      return;
    }
    if(millis() - start_time < start_delay_ms) {
      return;
    }

    // Ask the signals to move if not already. Multiple calls are fine.
    if (current->signal_state == State::DANGER) {
      current->signal->move_danger();
    }
    if (current->signal_state == State::CLEAR) {
      current->signal->move_clear();
    }
    if (current->signal->get_state() == current->signal_state || t != 0) {
      // Signal in correct state
      // Start waiting if not already
      if (t == 0) {
        t = millis();
      }
      if (current-elements == size - 1) {
        stop();
        return;
      } else if (millis() - t > current->time_delay_ms) {
        t = 0;
        current++;

        while (current->signal->get_state() == current->signal_state) {
          if (current-elements < size - 1) {
            current++;
          } else {
            stop();
            return;
          }
        }
      }
    }
  }
private:
  bool is_waiting = false;
  SequenceElement* current = nullptr;
  unsigned long t = 0;
  int start_delay_ms = 0;
  unsigned long start_time = 0;
  unsigned char size;
  SequenceElement* elements;
};

Signal * get_signal(int servo_number, BoardColour colour) {
  if (colour == BoardColour::YELLOW) {
    return signals + (servo_number - 1);
  }
  
  if (colour == BoardColour::ROSE) {
    return signals + (servo_number - 1) + 6;
  }
  if (colour == BoardColour::BLUE) {
    return signals + (servo_number - 1) + 14;
  }
}

const SequenceElement se13[] = {
  {get_signal(2, BoardColour::YELLOW), State::DANGER},
  {get_signal(1, BoardColour::YELLOW), State::DANGER},
  {get_signal(6, BoardColour::YELLOW), State::CLEAR},
};
const SequenceElement se12[] = {
  {get_signal(6, BoardColour::YELLOW), State::DANGER},
  {get_signal(3, BoardColour::YELLOW), State::DANGER},
  {get_signal(2, BoardColour::YELLOW), State::CLEAR},
  {get_signal(1, BoardColour::YELLOW), State::CLEAR},
};
const SequenceElement se11[] = {
  {get_signal(3, BoardColour::YELLOW), State::DANGER},
  {get_signal(4, BoardColour::YELLOW), State::CLEAR},
  {get_signal(7, BoardColour::ROSE), State::CLEAR},
};
const SequenceElement se10[] = {
  {get_signal(4, BoardColour::YELLOW), State::DANGER},
  {get_signal(5, BoardColour::ROSE), State::DANGER},
  {get_signal(6, BoardColour::ROSE), State::DANGER},
  {get_signal(7, BoardColour::ROSE), State::DANGER},
  {get_signal(3, BoardColour::YELLOW), State::CLEAR},
};
const SequenceElement se9[] = {
  {get_signal(4, BoardColour::ROSE), State::DANGER},
  {get_signal(3, BoardColour::YELLOW), State::DANGER},
  {get_signal(5, BoardColour::ROSE), State::CLEAR},
  {get_signal(6, BoardColour::ROSE), State::CLEAR},
};
const SequenceElement se8[] = {
  {get_signal(3, BoardColour::ROSE), State::DANGER},
  {get_signal(5, BoardColour::YELLOW), State::DANGER},
  {get_signal(5, BoardColour::ROSE), State::DANGER},
  {get_signal(6, BoardColour::ROSE), State::DANGER},
  {get_signal(4, BoardColour::ROSE), State::CLEAR},
};
const SequenceElement se7[] = {
  {get_signal(8, BoardColour::ROSE), State::DANGER},
  {get_signal(4, BoardColour::ROSE), State::DANGER},
  {get_signal(5, BoardColour::YELLOW), State::DANGER},
  {get_signal(3, BoardColour::ROSE), State::CLEAR},
};
const SequenceElement se6[] = {
  {get_signal(8, BoardColour::ROSE), State::DANGER},
  {get_signal(1, BoardColour::ROSE), State::DANGER},
  {get_signal(2, BoardColour::ROSE), State::CLEAR},
};
const SequenceElement se5[] = {
  {get_signal(8, BoardColour::ROSE), State::DANGER},
  {get_signal(2, BoardColour::ROSE), State::DANGER},
  {get_signal(1, BoardColour::ROSE), State::CLEAR},
};
const SequenceElement se4[] = {
  {get_signal(3, BoardColour::BLUE), State::DANGER},
  {get_signal(5, BoardColour::BLUE), State::DANGER},
  {get_signal(4, BoardColour::BLUE), State::CLEAR},
};
const SequenceElement seA3[] = {
  {get_signal(4, BoardColour::BLUE), State::DANGER},
  {get_signal(2, BoardColour::BLUE), State::DANGER},
  {get_signal(3, BoardColour::BLUE), State::CLEAR},
  {get_signal(7, BoardColour::BLUE), State::CLEAR},
};
const SequenceElement seA2[] = {
  {get_signal(3, BoardColour::BLUE), State::DANGER},
  {get_signal(4, BoardColour::BLUE), State::DANGER},
  {get_signal(1, BoardColour::BLUE), State::DANGER},
  {get_signal(7, BoardColour::BLUE), State::DANGER},
  {get_signal(6, BoardColour::BLUE), State::DANGER},
  {get_signal(2, BoardColour::BLUE), State::CLEAR},
  {get_signal(5, BoardColour::BLUE), State::CLEAR},
};
const SequenceElement seA1[] = {
  {get_signal(2, BoardColour::BLUE), State::DANGER},
  {get_signal(5, BoardColour::BLUE), State::DANGER},
  {get_signal(1, BoardColour::BLUE), State::CLEAR},
  {get_signal(6, BoardColour::BLUE), State::CLEAR},
};
const SequenceElement seA0[] = {
  {get_signal(3, BoardColour::ROSE), State::DANGER},
  {get_signal(4, BoardColour::ROSE), State::DANGER},
  {get_signal(2, BoardColour::ROSE), State::DANGER},
  {get_signal(1, BoardColour::ROSE), State::DANGER},
  {get_signal(8, BoardColour::ROSE), State::CLEAR},
  {get_signal(5, BoardColour::YELLOW), State::CLEAR},
};

const SequenceElement he1[] = {
  {get_signal(1, BoardColour::ROSE), State::DANGER, 0},
  {get_signal(2, BoardColour::ROSE), State::DANGER, 0},
  {get_signal(8, BoardColour::ROSE), State::DANGER, 0}, 
};
const SequenceElement he2[] = {
  {get_signal(3, BoardColour::ROSE), State::DANGER, 0},
  {get_signal(4, BoardColour::ROSE), State::DANGER, 0},
  {get_signal(5, BoardColour::YELLOW), State::DANGER, 0}, 
};
const SequenceElement he3[] = {
  {get_signal(6, BoardColour::ROSE), State::DANGER, 0},
};
const SequenceElement he4[] = {
  {get_signal(5, BoardColour::ROSE), State::DANGER, 2000},
};
const SequenceElement he5[] = {
  {get_signal(6, BoardColour::YELLOW), State::DANGER, 0},
  {get_signal(2, BoardColour::YELLOW), State::DANGER, 1000},
  {get_signal(1, BoardColour::YELLOW), State::DANGER, 0}, 
};
const SequenceElement he6[] = {
  {get_signal(7, BoardColour::ROSE), State::DANGER, 0},
};
const SequenceElement he7[] = {
  {get_signal(3, BoardColour::YELLOW), State::DANGER, 0},
  {get_signal(4, BoardColour::YELLOW), State::DANGER, 0},
};
const SequenceElement he8[] = {
  {get_signal(5, BoardColour::BLUE), State::DANGER, 0},
};
const SequenceElement he9[] = {
  {get_signal(7, BoardColour::BLUE), State::DANGER, 0},
};
const SequenceElement he10[] = {
  {get_signal(3, BoardColour::BLUE), State::DANGER, 0},
  {get_signal(4, BoardColour::BLUE), State::DANGER, 0},
};
const SequenceElement he11[] = {
  {get_signal(6, BoardColour::BLUE), State::DANGER, 0},
};
const SequenceElement he12[] = {
  {get_signal(1, BoardColour::BLUE), State::DANGER, 0},
  {get_signal(2, BoardColour::BLUE), State::DANGER, 0},
};


const Sequence point_sequences[] = {
  {se13, 3},
  {se12, 4},
  {se11, 3},
  {se10, 5},
  {se9, 4},
  {se8, 5},
  {se7, 4},
  {se6, 3},
  {se5, 3},
  {se4, 3},
  {seA3, 4},
  {seA2, 7},
  {seA1, 4},
  {seA0, 6},
};
const char point_triggers[] = {
  13,
  12,
  11,
  10,
  9,
  8,
  7,
  6,
  5,
  4,
  A3,
  A2,
  A1,
  A0  
};

// Last value in triple is start delay in ms
const Sequence hall_sequences[] = {
  {he1, 3, 2000},
  {he2, 3, 2000},
  {he3, 1, 2000},
  {he4, 1, 2000},
  {he5, 3, 2000},
  {he6, 1, 2000},
  {he7, 2, 2000},
  {he8, 1, 2000},
  {he9, 1, 2000},
  {he10, 2, 2000},
  {he11, 1, 2000},
  {he12, 2, 2000},
};

float hall_averages[12] = {};

void output_enable() {
  digitalWrite(A5, LOW);
}
void output_disable() {
  digitalWrite(A5, HIGH);
}


void setup() {
  // Multiplexer select pins
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);

  // Set up ADC
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);   // 128

  // OE
  pinMode(A5, OUTPUT);
  output_disable();
  
  for (auto& pin : point_triggers) {
    pinMode(pin, INPUT_PULLUP);
  }

  Serial.begin(115200);

  i2c.setWireTimeout(25000, true);  //25ms (25000us) is typical according to the wire.cpp source, reset on fault
  i2c.setClock(10000);
  i2c.begin();

  pwm0.begin();
  pwm0.setOscillatorFrequency(27000000);
  pwm0.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  pwm2.begin();
  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  Serial.println(F("Ready"));
}

void loop() {
  static int loop_count = 0;
  static bool startup = true;
  auto now = millis();
  auto dt = now - heartbeat;
  loop_count++;
  if (dt > 2000) {
    heartbeat = now;
    Serial.println(F("Heartbeat - version: hall effect"));
    Serial.print(F("Loop time: "));
    Serial.println(1.0 * dt / loop_count);
    loop_count = 0;
    output_enable();
    startup = false;
}

  // Check for sequences that want to change their signals
  static int last_point_trigger = 99;
  for (unsigned char i=0; i< sizeof(point_sequences)/sizeof(point_sequences[0]); i++) {
    // Check the triggers
    if (!digitalRead(point_triggers[i])) {
      point_sequences[i].enqueue();
      if (last_point_trigger != i) {
        Serial.print(F("Points triggered: "));
        Serial.println(point_triggers[i]*1);
        last_point_trigger = i;
      }
    }
  }


  for (const auto & sequence : point_sequences) {
    sequence.update();
  }
 
  for (int hall_index=0; hall_index < 12; hall_index++) {
    // Check the triggers
    // Triggers are sequential so no need to map them
    
    // Set the pins to sample the correct input
    digitalWrite(14, (hall_index & (1<<0))!=0);  // Multiplexer A0
    digitalWrite(15, (hall_index & (1<<1))!=0);  // Multiplexer A1
    digitalWrite(16, (hall_index & (1<<2))!=0);  // Multiplexer A2
    digitalWrite(17, (hall_index & (1<<3))!=0);  // Multiplexer A3

    if(startup) {
      analogRead(A4);
      hall_averages[hall_index] *= 0.5;
      hall_averages[hall_index] += analogRead(A4) * 0.5;
      continue;
    }
    
    int val = 0;
    int loops = 2;
    for (int j=0; j<loops; j++) {
      val += analogRead(A4);  // throw away the first read to let it stabilise
    }
    val = val/loops;
    //val = analogRead(A4);
    constexpr static int tol = (int)((3.7-2.5)/5.0*1024.0);  // 3.7V trigger
    static int last_hall_trigger = 99;
    //if (val > 512+tol or val < 512-tol) {  // Check in both directions in case magnets flipped
    if (val > hall_averages[hall_index] * 1.3 || val < hall_averages[hall_index] * 0.7) {  // Check in both directions in case magnets flipped
      hall_sequences[hall_index].enqueue();
      if (last_hall_trigger != hall_index) {
        Serial.print(F("Hall effect triggered: "));
        Serial.print(hall_index+1);
        Serial.print(F(" - "));
        Serial.println(val);
        last_hall_trigger = hall_index;
      }
    }
  }

  for (const auto & sequence : hall_sequences) {
    sequence.update();
  }

  // Now move the signals
  for (auto& signal : signals) {
    signal.update();
  }
}
