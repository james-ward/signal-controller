#include <Wire.h>
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

unsigned char debounce_counter = 0;

class Signal;

enum struct BoardColour {
  YELLOW,
  PINK,
  BLUE,
};

void setPosition(int servonum, Adafruit_PWMServoDriver& pwm, float pos_degrees) {
  float pulselen = map(pos_degrees, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servonum, 0, pulselen);
}

enum class State {
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
    auto elapsed = millis() - t;
    float pos;
    switch (state) {
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
        set_degrees(1.);
        break;
      case State::DANGER:
        set_degrees(0.);
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
  State state = State::DANGER;
  const unsigned int up_time_ms = 2000;
  const unsigned int down_time_ms = 750;
  const unsigned char clear_deg;
  const unsigned char danger_deg;

  void set_degrees(float pos) {
    if (pos > 1.0) {
      pos = 1.0;
    }
    if (pos < 0.0) {
      pos = 0.0;
    }
    float deg = (1.0* clear_deg - 1.0* danger_deg) * pos + 1.0 * danger_deg;
    setPosition(servo, pwm, deg);
  }
};

class SequenceElement {
public:
  Signal* signal;
  State signal_state;
  unsigned int time_delay_ms = 1000;

  SequenceElement(Signal* signal, State signal_state, float time_delay = 0.3): signal(signal), signal_state(signal_state), time_delay_ms(time_delay_ms) {}
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
  // pink
  {1, pwm1, 95, 55}, //+
  {2, pwm1, 105, 65}, //+
  {3, pwm1, 140, 60}, //+
  {4, pwm1, 80, 130}, //-
  {5, pwm1, 30, 70}, //- //10
  {6, pwm1, 45, 100}, //- 
  {7, pwm1, 60, 105}, //-
  {8, pwm1, 135, 90}, //+
  // blue
  {1, pwm2, 35, 70}, //-
  {2, pwm2, 135, 80}, //+
  {3, pwm2, 150, 90},  //+ //15
  {4, pwm2, 135, 85}, //+
  {5, pwm2, 70, 125}, //-
  {6, pwm2, 110, 85},  //+
  {7, pwm2, 83, 119}, //- //19
};


class Sequence {
public:
  Sequence(SequenceElement* elements, unsigned char size) : size{size},
    elements{elements} {}
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
    /*
    if (digitalRead(trigger) and debounce_counter > 20) {
      count++;
    } else {
      count = 0;
    }
    if ((
        (trigger >= A0 and count > 1) or (trigger < A0 and count > 0)
        ) and current == nullptr and not is_waiting) {
      Serial.print(trigger);
      Serial.println(" triggered");
      // start a cycle
      is_waiting = true;
      debounce_counter = 0;
      count = 0;
    }
    */
    if (is_waiting and not sequence_running) {
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
    }
    if (current == nullptr) {
      return;
    }

    // Ask the signals to move if not already. Multiple calls are fine.
    if (current->signal_state == State::DANGER) {
      current->signal->move_danger();
    }
    if (current->signal_state == State::CLEAR) {
      current->signal->move_clear();
    }
    if (current->signal->get_state() == current->signal_state or t != 0) {
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
  unsigned char size;
  SequenceElement* elements;
};

Signal * get_signal(int servo_number, BoardColour colour) {
  if (colour == BoardColour::YELLOW) {
    return signals + (servo_number - 1);
  }
  if (colour == BoardColour::PINK) {
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
  {get_signal(7, BoardColour::PINK), State::CLEAR},
};
const SequenceElement se10[] = {
  {get_signal(4, BoardColour::YELLOW), State::DANGER},
  {get_signal(5, BoardColour::PINK), State::DANGER},
  {get_signal(6, BoardColour::PINK), State::DANGER},
  {get_signal(7, BoardColour::PINK), State::DANGER},
  {get_signal(3, BoardColour::YELLOW), State::CLEAR},
};
const SequenceElement se9[] = {
  {get_signal(4, BoardColour::PINK), State::DANGER},
  {get_signal(3, BoardColour::YELLOW), State::DANGER},
  {get_signal(5, BoardColour::PINK), State::CLEAR},
  {get_signal(6, BoardColour::PINK), State::CLEAR},
};
const SequenceElement se8[] = {
  {get_signal(3, BoardColour::PINK), State::DANGER},
  {get_signal(5, BoardColour::YELLOW), State::DANGER},
  {get_signal(5, BoardColour::PINK), State::DANGER},
  {get_signal(6, BoardColour::PINK), State::DANGER},
  {get_signal(4, BoardColour::PINK), State::CLEAR},
};
const SequenceElement se7[] = {
  {get_signal(8, BoardColour::PINK), State::DANGER},
  {get_signal(4, BoardColour::PINK), State::DANGER},
  {get_signal(5, BoardColour::YELLOW), State::DANGER},
  {get_signal(3, BoardColour::PINK), State::CLEAR},
};
const SequenceElement se6[] = {
  {get_signal(8, BoardColour::PINK), State::DANGER},
  {get_signal(1, BoardColour::PINK), State::DANGER},
  {get_signal(2, BoardColour::PINK), State::CLEAR},
};
const SequenceElement se5[] = {
  {get_signal(8, BoardColour::PINK), State::DANGER},
  {get_signal(2, BoardColour::PINK), State::DANGER},
  {get_signal(1, BoardColour::PINK), State::CLEAR},
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
  {get_signal(3, BoardColour::PINK), State::DANGER},
  {get_signal(4, BoardColour::PINK), State::DANGER},
  {get_signal(2, BoardColour::PINK), State::DANGER},
  {get_signal(1, BoardColour::PINK), State::DANGER},
  {get_signal(8, BoardColour::PINK), State::CLEAR},
  {get_signal(5, BoardColour::YELLOW), State::CLEAR},
};

const SequenceElement he1[] = {
  {get_signal(1, BoardColour::PINK), State::DANGER, 0},
  {get_signal(2, BoardColour::PINK), State::DANGER, 0},
  {get_signal(8, BoardColour::PINK), State::DANGER, 0}, 
};
const SequenceElement he2[] = {
  {get_signal(3, BoardColour::PINK), State::DANGER, 0},
  {get_signal(4, BoardColour::PINK), State::DANGER, 0},
  {get_signal(5, BoardColour::YELLOW), State::DANGER, 0}, 
};
const SequenceElement he3[] = {
  {get_signal(6, BoardColour::PINK), State::DANGER, 0},
};
const SequenceElement he4[] = {
  {get_signal(5, BoardColour::PINK), State::DANGER, 0},
};
const SequenceElement he5[] = {
  {get_signal(6, BoardColour::YELLOW), State::DANGER, 0},
  {get_signal(2, BoardColour::YELLOW), State::DANGER, 1000},
  {get_signal(1, BoardColour::YELLOW), State::DANGER, 0}, 
};
const SequenceElement he6[] = {
  {get_signal(7, BoardColour::PINK), State::DANGER, 0},
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

const Sequence hall_sequences[] = {
  {he1, 3},
  {he2, 3},
  {he3, 1},
  {he4, 1},
  {he5, 3},
  {he6, 1},
  {he7, 2},
  {he8, 1},
  {he9, 1},
  {he10, 2},
  {he11, 1},
  {he12, 2},
};

void output_enable() {
  digitalWrite(A5, HIGH);
}
void output_disable() {
  digitalWrite(A5, LOW);
}


void setup() {
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(A4, INPUT);

  // OE
  pinMode(A5, OUTPUT);
  output_disable();
  
  for (auto& pin : point_triggers) {
    pinMode(pin, INPUT);
  }

  Serial.begin(115200);

  i2c.setWireTimeout(25000, true);  //25ms (25000us) is typical according to the wire.cpp source, reset on fault

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
  bool send_heartbeat = false;
  if (millis() - heartbeat > 2000) {
    heartbeat = millis();
    send_heartbeat = true;
  }
  // Check for sequences that want to change their signals
  for (unsigned char i=0; i< sizeof(point_sequences)/sizeof(point_sequences[0]); i++) {
    // Check the triggers
    if (debounce_counter > 20) {
      if (digitalRead(point_triggers[i])) {
        point_sequences[i].enqueue();
        debounce_counter = 0;
      }
    }
     
    point_sequences[i].update();
  }

  // Check for hall effect triggers to set signals back to danger
  for (unsigned char i=0; i< sizeof(hall_sequences)/sizeof(hall_sequences[0]); i++) {
    // Check the triggers
    // Triggers are sequential so no need to map them
    
    // Set the pins to sample the correct input
    digitalWrite(3, (i & (1<<0))!=0);  // Multiplexer A0
    digitalWrite(2, (i & (1<<1))!=0);  // Multiplexer A1
    digitalWrite(1, (i & (1<<2))!=0);  // Multiplexer A2
    digitalWrite(0, (i & (1<<3))!=0);  // Multiplexer A3
    
    auto val = analogRead(A4);
    int tol = (int)((3.7-2.5)*1024.0);  // 3.7V trigger
    if (val > 512+tol or val < 512-tol) {  // Check in both directions in case magnets flipped
      hall_sequences[i].enqueue(); 
    }
    hall_sequences[i].update();
  }

  
  // Now move the signals
  for (auto& signal : signals) {
    signal.update();
  }
  output_enable();
  
  if (send_heartbeat) {
    Serial.println("Heartbeat");
  }

  debounce_counter = min(debounce_counter + 1, 200);
}
