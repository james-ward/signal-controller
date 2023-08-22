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

unsigned int debounce_counter = 0;

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
  Signal(const int servo_number, Adafruit_PWMServoDriver& pwm, const float clear_degrees, const float danger_degrees) :
  servo(servo_number), pwm(pwm), clear_deg(clear_degrees), danger_deg(danger_degrees) {}
  void update() {
    float elapsed = (millis() - t) / 1000.0;
    switch (state) {
      case State::MOVING_CLEAR:
        if (elapsed < up_time / 2) {
          pos = (-cos(elapsed / (up_time/2) * 3.14159) + 1.0) / 4.0;
        } else {
          pos = 0.5 + (-cos((elapsed - up_time/2) / (up_time/2) * 3.14159) + 1.0) / 4.0;
        }
        if (elapsed > up_time) {
          state = State::CLEAR;
        }
        break;
      case State::MOVING_DANGER:
        if (elapsed < 0.70 * down_time) {
          pos = (cos(elapsed / (0.70 * down_time) * 3.14159/2.0));
        } else {
          pos = (1.0-cos((elapsed - 0.70 * down_time) / (0.30 * down_time) * 3.14159 * 2.0)) / 2.0 * 0.25;
        }
        if (elapsed > down_time) {
          state = State::DANGER;
        }
        break;
      case State::CLEAR:
      case State::DANGER:
        break;
    }
    set_degrees();
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
  int servo;
private:
  unsigned long t = 0;
  State state = State::DANGER;
  const float up_time = 2.0;
  const float down_time = 0.75;
  float pos = 0.0;
  const float clear_deg;
  const float danger_deg;

  void set_degrees() {
    if (pos > 1.0) {
      pos = 1.0;
    }
    if (pos < 0.0) {
      pos = 0.0;
    }
    float deg = (clear_deg - danger_deg) * pos + danger_deg;
    setPosition(servo, pwm, deg);
  }
};

class SequenceElement {
public:
  Signal* signal;
  State signal_state;
  float time_delay = 1.0;

  SequenceElement(Signal* signal, State signal_state, float time_delay = 0.3): signal(signal), signal_state(signal_state), time_delay(time_delay) {}
};


Signal signals[] = {
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
  Sequence(const int trigger, SequenceElement* elements, size_t size) : trigger{trigger}, size{size},
    elements{elements} {
    pinMode(trigger, INPUT);  
  }
  void stop() {
    current = nullptr;
    sequence_running = false;
    is_waiting = false;
    t = 0;
  }

  void update() {
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
    if (is_waiting and not sequence_running) {
      // Find the first element that is in the wrong state
      current = elements;
      while (current->signal->get_state() == current->signal_state) {
        if (current-elements < size - 1) {
          current++;
        } else {
          Serial.print(trigger);
          Serial.println(" no action required");
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
        Serial.print(trigger);
        Serial.print(": ");
        Serial.print(current->signal - signals);
        Serial.println(" correct state");
        t = millis();
      }
      if (current-elements == size - 1) {
        Serial.print(trigger);
        Serial.println(" done");
        stop();
        return;
      } else if ((millis() - t) / 1000.0 > current->time_delay) {
        Serial.print(trigger);
        Serial.println(" delay over");

        t = 0;

        current++;
        Serial.print(trigger);
        Serial.print(": ");
        Serial.print(current->signal - signals);
        Serial.println(" next step");

        while (current->signal->get_state() == current->signal_state) {
          if (current-elements < size - 1) {
            current++;
            Serial.print(trigger);
            Serial.print(": ");
            Serial.print(current->signal - signals);
            Serial.println(" skipping next step");
          } else {
            Serial.print(trigger);
            Serial.println(" done");
            stop();
            return;
          }
        }
      }
    }
  }
private:
  int trigger;
  bool is_waiting = false;
  SequenceElement* current = nullptr;
  unsigned long t = 0;
  unsigned int count = 0;
  size_t size;
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

SequenceElement se13[] = {
  {get_signal(2, BoardColour::YELLOW), State::DANGER},
  {get_signal(1, BoardColour::YELLOW), State::DANGER},
  {get_signal(6, BoardColour::YELLOW), State::CLEAR},
};
SequenceElement se12[] = {
  {get_signal(6, BoardColour::YELLOW), State::DANGER},
  {get_signal(3, BoardColour::YELLOW), State::DANGER},
  {get_signal(2, BoardColour::YELLOW), State::CLEAR},
  {get_signal(1, BoardColour::YELLOW), State::CLEAR},
};
SequenceElement se11[] = {
  {get_signal(3, BoardColour::YELLOW), State::DANGER},
  {get_signal(4, BoardColour::YELLOW), State::CLEAR},
  {get_signal(7, BoardColour::PINK), State::CLEAR},
};
SequenceElement se10[] = {
  {get_signal(4, BoardColour::YELLOW), State::DANGER},
  {get_signal(5, BoardColour::PINK), State::DANGER},
  {get_signal(6, BoardColour::PINK), State::DANGER},
  {get_signal(7, BoardColour::PINK), State::DANGER},
  {get_signal(3, BoardColour::YELLOW), State::CLEAR},
};
SequenceElement se9[] = {
  {get_signal(4, BoardColour::PINK), State::DANGER},
  {get_signal(3, BoardColour::YELLOW), State::DANGER},
  {get_signal(5, BoardColour::PINK), State::CLEAR},
  {get_signal(6, BoardColour::PINK), State::CLEAR},
};
SequenceElement se8[] = {
  {get_signal(3, BoardColour::PINK), State::DANGER},
  {get_signal(5, BoardColour::YELLOW), State::DANGER},
  {get_signal(5, BoardColour::PINK), State::DANGER},
  {get_signal(6, BoardColour::PINK), State::DANGER},
  {get_signal(4, BoardColour::PINK), State::CLEAR},
};
SequenceElement se7[] = {
  {get_signal(8, BoardColour::PINK), State::DANGER},
  {get_signal(4, BoardColour::PINK), State::DANGER},
  {get_signal(5, BoardColour::YELLOW), State::DANGER},
  {get_signal(3, BoardColour::PINK), State::CLEAR},
};
SequenceElement se6[] = {
  {get_signal(8, BoardColour::PINK), State::DANGER},
  {get_signal(1, BoardColour::PINK), State::DANGER},
  {get_signal(2, BoardColour::PINK), State::CLEAR},
};
SequenceElement se5[] = {
  {get_signal(8, BoardColour::PINK), State::DANGER},
  {get_signal(2, BoardColour::PINK), State::DANGER},
  {get_signal(1, BoardColour::PINK), State::CLEAR},
};
SequenceElement se4[] = {
  {get_signal(3, BoardColour::BLUE), State::DANGER},
  {get_signal(5, BoardColour::BLUE), State::DANGER},
  {get_signal(4, BoardColour::BLUE), State::CLEAR},
};
SequenceElement seA3[] = {
  {get_signal(4, BoardColour::BLUE), State::DANGER},
  {get_signal(2, BoardColour::BLUE), State::DANGER},
  {get_signal(3, BoardColour::BLUE), State::CLEAR},
  {get_signal(7, BoardColour::BLUE), State::CLEAR},
};
SequenceElement seA2[] = {
  {get_signal(3, BoardColour::BLUE), State::DANGER},
  {get_signal(4, BoardColour::BLUE), State::DANGER},
  {get_signal(1, BoardColour::BLUE), State::DANGER},
  {get_signal(7, BoardColour::BLUE), State::DANGER},
  {get_signal(6, BoardColour::BLUE), State::DANGER},
  {get_signal(2, BoardColour::BLUE), State::CLEAR},
  {get_signal(5, BoardColour::BLUE), State::CLEAR},
};
SequenceElement seA1[] = {
  {get_signal(2, BoardColour::BLUE), State::DANGER},
  {get_signal(5, BoardColour::BLUE), State::DANGER},
  {get_signal(1, BoardColour::BLUE), State::CLEAR},
  {get_signal(6, BoardColour::BLUE), State::CLEAR},
};
SequenceElement seA0[] = {
  {get_signal(3, BoardColour::PINK), State::DANGER},
  {get_signal(4, BoardColour::PINK), State::DANGER},
  {get_signal(2, BoardColour::PINK), State::DANGER},
  {get_signal(1, BoardColour::PINK), State::DANGER},
  {get_signal(8, BoardColour::PINK), State::CLEAR},
  {get_signal(5, BoardColour::YELLOW), State::CLEAR},
};

Sequence sequences[] = {
  {13, se13, 3},
  {12, se12, 4},
  {11, se11, 3},
  {10, se10, 5},
  {9, se9, 4},
  {8, se8, 5},
  {7, se7, 4},
  {6, se6, 3},
  {5, se5, 3},
  {4, se4, 3},
  {A3, seA3, 4},
  {A2, seA2, 7},
  {A1, seA1, 4},
  {A0, seA0, 6},
};


void setup() {
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

  Serial.println("Ready");
}


void loop() {
  bool send_heartbeat = false;
  if (millis() - heartbeat > 2000) {
    heartbeat = millis();
    send_heartbeat = true;
  }
  // Check for sequences that want to change their signals
  for (auto& sequence : sequences) {
    sequence.update();
  }
  // Now move the signals
  for (auto& signal : signals) {
    signal.update();
  }
  
  if (send_heartbeat) {
    Serial.println("Heartbeat");
  }

  debounce_counter = min(debounce_counter + 1, 100000);
}
