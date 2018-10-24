/* Copyright (c) 2018 John Cronin <jncronin@tysos.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Teensy++ 2.0 code for the djhero mod
 *  
 *  Implements a USB joystick and mouse and responds to
 *  USB serial inputs
 *  
 *  btn_info defines the mapping between pins, joystick buttons and
 *  keyboard buttons.  It includes a debouncer from the Bounce2 
 *  library for each pin.  Set joy_btn or key to -1 if that particular
 *  pin does not connect to a joystick or keyboard button.
 *  
 *  dpad_info defines the input pins for the DPAD, in the order
 *  left, right, up, down within dpad[].  A debouncer is also included
 *  for these buttons.  They are mapped to the joystick X and Y axes.
 *  
 *  All input pins have pullups enabled, thus just need connecting to
 *  ground via a switch to function.  Extra ground pins are provided
 *  on pins 9, 12, 13 and 22 to help this.
 *  
 *  Analog input 0 is mapped directly to joystick Zrotate, as well as
 *  being converted to LEFT/RIGHT keypresses using a custom debouncer
 *  to handle noise on the input signal
 *  
 *  Pin 24 is an output pin designed to connect to an LED.  By default
 *  it flashes once per second using fade-in/out effects.  This can
 *  be altered using the serial interface by sending single characters:
 *  
 *  '0'-'9' (ASCII characters) - set the LED to fixed mode at the
 *    appropriate brightness
 *  'f' or 'F' - start the LED flashing
 *  'a', 'b', 'c' - rotary pot window size
 */

#include <Bounce2.h>

struct btn_info
{
  int pin;
  int joy_btn;
  int key;
  bool old_state;
  Bounce b;
};

struct dpad_info
{
  int pin;
  Bounce b;
};

struct btn_info btns[] = {
  { 3, 1, (int)KEY_ENTER, true },       // A
  { 2, 2, (int)KEY_BACKSPACE, true},    // B
  { 1, 3, -1, true },                   // X
  { 0, 4, -1, true },                   // Y
  { 10, 5, (int)KEY_ESC, true },        // Back
  { 11, 6, -1, true },                  // Start
  { 23, 7, (int)KEY_ENTER, true },      // Big button
  { 15, -1, (int)KEY_LEFT, true },      // DPAD left
  { 14, -1, (int)KEY_RIGHT, true },     // DPAD right
  { 17, -1, (int)KEY_UP, true },        // DPAD up
  { 16, -1, (int)KEY_DOWN, true },      // DPAD down

  { 20, -1, (int)KEY_R, true },         // turntable
  { 19, -1, (int)KEY_B, true },         // turntable
  { 18, -1, (int)KEY_G, true },         // turntable
};

struct dpad_info dpad[] = {
  { 15 },      // LEFT
  { 14 },      // RIGHT
  { 17 },      // UP
  { 16 },      // DOWN
};

int btn_count;

bool big_is_flashing = true;

unsigned long rp_debounce_start = 0;
const unsigned long rp_debounce_time = 10;
int rp_window_width = 1024/24;
int rp_window_centre = 0;
int rp_last_window_centre = 0;

unsigned long sp_debounce_start = 0;
const int sp_window_width = 1024/32;
int sp_window_centre = 0;

unsigned long vp_debounce_start = 0;
const int vp_window_width = 1024/100;
int vp_window_centre = 0;

int mouse_last_x = 0;
int mouse_last_y = 0;
unsigned long mouse_last_update_time = 0;

void setup() {
  // put your setup code here, to run once:

  // generate some extra ground lines
  pinMode(9, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  digitalWrite(9, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(21, LOW);
  digitalWrite(22, LOW);

  // Big button LED
  pinMode(25, OUTPUT);
  digitalWrite(25, LOW);
  pinMode(24, OUTPUT);
  digitalWrite(24, HIGH);

  // Generate debouncers for buttons
  btn_count = sizeof(btns) / sizeof(struct btn_info);

  for(int i = 0; i < btn_count; i++)
  {
    btns[i].b = Bounce();
    btns[i].b.attach(btns[i].pin, INPUT_PULLUP);
    btns[i].b.interval(10);
  }
  for(int i = 0; i < 4; i ++)
  {
    dpad[i].b = Bounce();
    dpad[i].b.attach(dpad[i].pin, INPUT_PULLUP);
    dpad[i].b.interval(10);
  }

  // Initialise rotary pot
  rp_debounce_start = millis();
  rp_window_centre = analogRead(0);
  rp_last_window_centre = rp_window_centre;

  // USB serial setup (baudrate is ignored for Teensy)
  Serial.begin(9600);

  // Big button is set to flash
  big_is_flashing = true;

  // Mouse automatically reports its position every 100 ms
  mouse_last_update_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
   
  for(int i = 0; i < btn_count; i++)
  {
    btns[i].b.update();

    bool cur_val = btns[i].b.read();
    if(cur_val != btns[i].old_state)
    {
      // a change has occurred
      
      // Button presses
      if(btns[i].joy_btn != -1)
        Joystick.button(btns[i].joy_btn, cur_val ? 0 : 1);

      // Keyboard presses
      if(btns[i].key != -1)
      {
        if(cur_val)
          Keyboard.release(btns[i].key);
        else
          Keyboard.press(btns[i].key);
      }

      btns[i].old_state = cur_val;
    }
  }

  // handle dpad
  for(int i = 0; i < 4; i++)
  {
    dpad[i].b.update();
  }

  bool left = dpad[0].b.read();
  bool right = dpad[1].b.read();
  bool up = dpad[2].b.read();
  bool down = dpad[3].b.read();

  if(left == right)
    Joystick.X(512);
  else if(left == 0)
    Joystick.X(0);
  else
    Joystick.X(1023);

  if(up == down)
    Joystick.Y(512);
  else if(up == 0)
    Joystick.Y(0);
  else
    Joystick.Y(1023);

  // Test rotary pot
  Joystick.Zrotate(analogRead(0));

  // Debounce slide pot
  unsigned long cur_time = millis();
  int cur_sp = analogRead(1);
  int sp_diff = cur_sp - sp_window_centre;
  int mouse_bin = mouse_last_x;
  if(sp_diff > sp_window_width || sp_diff < -sp_window_width)
  {
    sp_window_centre = cur_sp;
    sp_debounce_start = cur_time;
  }
  else
  {
    // If timer has elapsed, we have a new center
    unsigned long time_diff = cur_time - sp_debounce_start;
  
    if(time_diff > rp_debounce_time)
    {
      mouse_bin = sp_window_centre / 32 + 1;
      sp_window_centre = cur_sp;
    }
  }

  // Debounce volume pot
  int cur_vp = analogRead(2);
  int vp_diff = cur_vp - vp_window_centre;
  int mouse_y_bin = mouse_last_y;
  if(vp_diff > vp_window_width || vp_diff < -vp_window_width)
  {
    vp_window_centre = cur_vp;
    vp_debounce_start = cur_time;
  }
  else
  {
    // If timer has elapsed, we have a new center
    unsigned long time_diff = cur_time - vp_debounce_start;
  
    if(time_diff > rp_debounce_time)
    {
      mouse_y_bin = vp_window_centre / 100 + 1;
      vp_window_centre = cur_vp;
    }    
  }
  if(mouse_bin != mouse_last_x || mouse_y_bin != mouse_last_y || cur_time > (mouse_last_update_time + 100))
  {
    Mouse.move(mouse_bin, mouse_y_bin);
    mouse_last_x = mouse_bin;
    mouse_last_y = mouse_y_bin;
    mouse_last_update_time = cur_time;
  }

  // Handle rotary pot as a left/right key press
  int cur_rp = analogRead(0);
  int diff = cur_rp - rp_window_centre;

  // Handle wrap-around 0 mark
  if(diff < -512)
    diff += 1024;
  if(diff > 512)
    diff -= 1024;

  // If we have moved outside the window, reset the timer
  if(diff > rp_window_width || diff < -rp_window_width)
  {
    rp_window_centre = cur_rp;
    rp_debounce_start = cur_time;
  }
  else
  {
    // If timer has elapsed, we have a new center
    unsigned long time_diff = cur_time - rp_debounce_start;
  
    if(time_diff > rp_debounce_time)
    {
      // decide on whether we have moved sufficiently to generate a key press
      int diff2 = rp_window_centre - rp_last_window_centre;

      if(diff2 < -512)
        diff2 += 1024;
      if(diff2 > 512)
        diff2 -= 1024;

      if(diff2 > rp_window_width)
      {
        Keyboard.press(KEY_RIGHT);
        Keyboard.release(KEY_RIGHT);
        rp_last_window_centre = rp_window_centre;
        rp_window_centre = cur_rp;
      }
      else if(diff2 < -rp_window_width)
      {
        Keyboard.press(KEY_LEFT);
        Keyboard.release(KEY_LEFT);
        rp_last_window_centre = rp_window_centre;
        rp_window_centre = cur_rp;
      }
    }
  }

  // Serial interface function
  if(Serial.available())
  {
    int svar = Serial.read();
    if(svar >= '0' && svar <= '9')
    {
      svar -= '0';
      analogWrite(24, 255 / 9 * svar);
      big_is_flashing = false;
    }
    if(svar == 'f' || svar == 'F')
    {
      big_is_flashing = true;
    }
    if(svar == 'a' || svar == 'A')
    {
      rp_window_width = 1024 / 16;
    }
    if(svar == 'b' || svar == 'B')
    {
      rp_window_width = 1024 / 24;
    }
    if(svar == 'c' || svar == 'C')
    {
      rp_window_width = 1024 / 32;
    }
  }

  // Big button flash
  if(big_is_flashing)
  {
    // 1 hz flashes
    // analog out has 255 steps, so up and down is ~512
    // make this 500 for simplicity
    unsigned long ms = millis() % 1000;
    if(ms < 500)
      analogWrite(24, ms / 2);
    else
      analogWrite(24, (1000 - ms) / 2);
  }
}
