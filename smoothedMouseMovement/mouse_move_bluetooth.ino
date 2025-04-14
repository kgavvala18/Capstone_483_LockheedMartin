
// #include <tinyekf.hpp>

/*this is an implementation of mouse control using the gyroscope of the LSM6DSOX IMU
An Extended Kalman Filter is used to smooth the acceleration values
which are scaled and used directly for control input
the KB2040 board was used to test this implmentation using usb hid and we are still
converting this to use on the feather nRF52840 sense board with BLe HID
  */

#define EKF_N 7 //  States: Y angular disp, Y ang vel, Z ang disp, Z ang vel, pointer, middle, thumb
// Want 9: Y lin Acc, Y lin Vel, Z lin Acc, Z lin Vel, Y ang Vel, Z ang Vel, pointer, middle, thumb
#define EKF_M 5 //  Measurements: Y ang vel, Z ang vel, pointer, middle, thumb
// Want 7: Y lin Acc, Z lin Acc, Y ang Vel, Z ang Vel, pointer, middle, thumb

#include <bluefruit.h>
// #include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <tinyekf.h>
// #include <Mouse.h>
#include <gesture.h>
#include <math.h>
#include <Adafruit_TinyUSB.h>

BLEDis bledis;
BLEHidAdafruit blehid;

static const float EPS = 1.5e-6;

// static const float Q[EKF_N * EKF_N] = {

//     EPS, 0, 0, 0,
//     0, EPS * 10, 0, 0,
//     0, 0, EPS, 0,
//     0, 0, 0, EPS * 10};
static const float Q[EKF_N * EKF_N] = {

    EPS, 0, 0, 0, 0, 0, 0,
    0, EPS * 10, 0, 0, 0, 0, 0,
    0, 0, EPS, 0, 0, 0, 0,
    0, 0, 0, EPS * 10, 0, 0, 0,
    0, 0, 0, 0, EPS * 10, 0, 0,
    0, 0, 0, 0, 0, EPS * 10, 0,
    0, 0, 0, 0, 0, 0, EPS * 10};

// static const float R[EKF_M * EKF_M] = {
//     // 2x2

//     0.0005,
//     0,
//     0,
//     0.0005,
// };

static const float ER[EKF_M * EKF_M] = {
    // USED TO BE THE R MATRIX
    0.0005,
    0,
    0,
    0,
    0,
    0,
    0.0005,
    0,
    0,
    0,
    0,
    0,
    0.0005,
    0,
    0,
    0,
    0,
    0,
    0.0005,
    0,
    0,
    0,
    0,
    0,
    0.0005,
};

// So process model Jacobian is identity matrix
// static const float F[EKF_N*EKF_N] = {   //
//     1, 1, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 1, 1,
//     1, 1, 0, 1
// };

// static const float H[EKF_M * EKF_N] = {
//     0,
//     1,
//     0,
//     0,
//     0,
//     0,
//     0,
//     1,
// };

static const float H[EKF_M * EKF_N] = {
    0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 1};

static ekf_t _ekf;

Adafruit_LSM6DSOX sox;
unsigned long myTime;
float prev;
float curr;
float dt = 0.01;

int sensorpin0 = A0; // ring flex sensor pin
int sensorpin1 = A1; // middle flex sensor pin
int sensorpin2 = A2; // thumb sensor pin
int sensorpin3 = A3; // pointer sensor pin
int sensorpin4 = A4; // pinky sensor pin

int middle;  // ring
int pointer; // middle
int thumb;   // middle
int ring;    // middle
int pinky;   // middle

int leftclick = 0;
int leftactive = 0;
int rightclick = 0;
int laser = 0;
// float F[EKF_N*EKF_N];

float prev_pointer_1 = -1; // Grant - added to toggle mouse movement when moving fingers
float prev_middle_1 = -1;

// needed for gesture key presses
uint8_t emptyKeycode[6] = {0};
uint8_t activeKey[6] = {0};

bool mouseEnabled = true;
bool toggleMouse = false;

constexpr Gesture LEFT_CLICK_GESTURE = I;
constexpr Gesture RIGHT_CLICK_GESTURE = M;
constexpr Gesture DRAG_GESTURE = TI;
constexpr Gesture LASER_GESTURE = T;
constexpr Gesture DISABLE_MOUSE_GESTURE = TIMRP;
constexpr Gesture SNIP_GESTURE = TMRP;
constexpr Gesture ALT_F4_GESTURE = TIRP;
constexpr Gesture ALT_TAB_GESTURE = MRP;
constexpr Gesture ALT_SHIFT_TAB_GESTURE = IRP;
constexpr Gesture ZOOM_GESTURE = TP;
constexpr Gesture SCROLL_GESTURE = TRP;

enum GestureState
{
  IDLE,
  LEFT_CLICK_EVENT,
  RIGHT_CLICK_EVENT,
  DRAG_EVENT,
  LASER,
  SNIP,
  ALT_F4,
  ALT_TAB,
  ALT_SHIFT_TAB,
  DISABLE_MOUSE,
  ZOOM,
  SCROLL
};

GestureState gestureState = IDLE;

// the setup routine runs once when you press reset:
void setup()
{

  Serial.begin(115200);
  while (!Serial)
  {
    Serial.println("cannot proceed!");
    delay(10);
  }

  Serial.println("Motion Control Glove - Starting");
  const float Pdiag[EKF_N] = {0.001, 0.001, 0.001, 0.001, 0.001};

  ekf_initialize(&_ekf, Pdiag);

  if (!sox.begin_SPI(1))
  {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSOX chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("LSM6DSOX Found!");
  prev = 0;
  // Configure accelerometer/gyroscope
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  sox.setAccelDataRate(LSM6DS_RATE_208_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_208_HZ);

  configBluetooth();

  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // BLE HID
  blehid.begin();

  // Set up and start advertising
  startAdv();
  pinMode(A5, OUTPUT); // move the laser to a digital pin instead of analog
  digitalWrite(A5, LOW);
}

// the loop routine runs over and over again forever:
void loop()
{
  curr = millis();

  dt = (curr - prev) / 1000.0000;
  prev = curr;

  middle = analogRead(sensorpin0);
  pointer = analogRead(sensorpin1);

  thumb = analogRead(sensorpin2);
  ring = analogRead(sensorpin3);
  pinky = analogRead(sensorpin4);

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  sox.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y + 0.32;
  float az = accel.acceleration.z - 10.03;

  float gx = gyro.gyro.x;
  float gy = 0.01 + gyro.gyro.y;
  float gz = 0.01 + gyro.gyro.z;

  const float z[EKF_M] = {gy, gz, pointer, middle, thumb};

  const float F[EKF_N * EKF_N] = {
      1, dt, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0,
      0, 0, 1, dt, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 1};

  // Process model f(x)
  const float fx[EKF_N] = {_ekf.x[0] + dt * _ekf.x[1], _ekf.x[1], _ekf.x[2] + dt * _ekf.x[3], _ekf.x[3], _ekf.x[4], _ekf.x[5], _ekf.x[6]}; // velocity y , velocity z

  // Run the prediction step of the eKF
  ekf_predict(&_ekf, fx, F, Q);

  const float hx[EKF_M] = {_ekf.x[1], _ekf.x[3], _ekf.x[4], _ekf.x[5], _ekf.x[6]};
  //   hx[2] = .9987 * this->x[1] + .001;

  // const float hx[EKF_M] = {_ekf.x[0], _ekf.x[1] };

  // Run  theupdaet step
  ekf_update(&_ekf, z, hx, H, ER);

  // }
  // else{
  //   vx = 0;
  //   vy = 0;
  // };

  // scale the acceleration values by 20
  /*the scaling factor was empiricaly determined. We need to find why this works
  to see if there may be a more optimal value*/
  float my = 20 * _ekf.x[1];
  float mz = -20 * _ekf.x[3];

  float pointer_1 = _ekf.x[4];
  float middle_1 = _ekf.x[5];
  float thumb_1 = _ekf.x[6];
  float pinky_1 = pinky;
  float ring_1 = ring;

  if (mouseEnabled)
  {
    blehid.mouseMove(mz, my);
  }

  ////
  ////GESTURE TESTING

  Serial.println(gesture(thumb_1, pointer_1, middle_1, ring_1, pinky_1));

  Gestures gesture_ = gesture(thumb, pointer_1, middle_1, ring, pinky);

  // this section of code handles click logic
  // we use rightclick and leftclick to manage states
  // if (rightclick)
  // {
  //   if (gesture_ == M) // to use gesture input use the gesture enum: m for middle
  //   {
  //     // Serial.println("click!: " + String(sensor0));
  //     rightclick = 0;
  //     blehid.mouseButtonRelease(MOUSE_BUTTON_RIGHT);
  //   }
  // }
  // else
  // {
  //   if (gesture_ == M)  // to use gesture input use the gesture enum: m for middle
  //   {
  //     // Serial.println("click!: " + String(sensor0));
  //     rightclick = 1;
  //     blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
  //     // Small delay to simulate a real click
  //   }
  // }

  switch (gestureState)
  {
  case IDLE:
    if (gesture_ == DRAG_GESTURE)
    {
      blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
      gestureState = DRAG_EVENT;
    }
    else if (gesture_ == LEFT_CLICK_GESTURE)
    {
      blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
      gestureState = LEFT_CLICK_EVENT;
      mouseEnabled = false;
    }
    else if (gesture_ == RIGHT_CLICK_GESTURE)
    {
      blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
      gestureState = RIGHT_CLICK_EVENT;
      mouseEnabled = false;
    }
    else if (gesture_ == LASER_GESTURE)
    {
      mouseEnabled = false;
      digitalWrite(A5, HIGH);
      gestureState = LASER;
    }
    else if (gesture_ == DISABLE_MOUSE_GESTURE)
    {
       toggleMouse = !toggleMouse;
       mouseEnabled = false;
       gestureState = DISABLE_MOUSE;
    }
     else if (gesture_ == SNIP_GESTURE)
     {
        activeKey[0] = 0x16;
        blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0x28, activeKey);
        gestureState = SNIP;
     }
     else if (gesture_ == ALT_F4_GESTURE)
     {
        mouseEnabled = false;
        activeKey[0] = 0x3D;
        blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0x04, activeKey);
        gestureState = ALT_F4;
     }
     else if (gesture_ == ALT_TAB_GESTURE)
     {
        mouseEnabled = false;
        activeKey[0] = 0x2B;
        blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0x04, activeKey);
        gestureState = ALT_TAB;
     }
     else if (gesture_ == ALT_SHIFT_TAB_GESTURE)
     {
        mouseEnabled = false;
        activeKey[0] = 0x2B;
        blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0x24, activeKey);
        gestureState = ALT_SHIFT_TAB;
     }
     else if (gesture_ == SCROLL_GESTURE)
     {
        mouseEnabled = false;

        // scroll up
        if (ay >= 1)
        {
          blehid.mouseScroll(1);
          gestureState = SCROLL;
        }

        // scroll down
        else
        {
          blehid.mouseScroll(-1);
          gestureState = SCROLL;
        }
     }
     else if (gesture_ == ZOOM_GESTURE)
     {
        mouseEnabled = false;

        // zoom in
        if (ay >= 1)
        {
          blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0x01, emptyKeycode); // Hold Ctrl (0x01 = Left Ctrl) is modifier for control
          blehid.mouseScroll(-1);
          gestureState = ZOOM;
        }

        // zoom out
        else
        {
          blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0x01, emptyKeycode); // Hold Ctrl (0x01 = Left Ctrl) is modifier for control
          blehid.mouseScroll(1);
          gestureState = ZOOM;
        }
     }
    break;

  case LEFT_CLICK_EVENT:
    if (gesture_ == NONE)
    {
      mouseEnabled = true;
      blehid.mouseButtonRelease(MOUSE_BUTTON_LEFT);
      gestureState = IDLE;
    }
    else if (gesture_ == DRAG_GESTURE)
    {
      mouseEnabled = true;
      gestureState = DRAG_EVENT;
    }
    break;

  case RIGHT_CLICK_EVENT:
    if (gesture_ == NONE)
    {
      mouseEnabled = true;
      blehid.mouseButtonRelease(MOUSE_BUTTON_RIGHT);
      gestureState = IDLE;
    }
    break;

  case DRAG_EVENT:
    if (gesture_ == NONE)
    {
      blehid.mouseButtonRelease(MOUSE_BUTTON_LEFT);
      gestureState = IDLE;
    }
    break;
   
  case LASER:
   if (gesture_ == NONE)
    {
      mouseEnabled = true;
      digitalWrite(A5, LOW);
      gestureState = IDLE;
    }
    else if (gesture_ == DRAG_GESTURE)
    {
      mouseEnabled = true;
      digitalWrite(A5, LOW);
      gestureState = LEFT_CLICK_EVENT;
    }
    break;
   
   case DISABLE_MOUSE:
     if (!toggleMouse)
     {
        mouseEnabled = true;
     }
     gestureState = IDLE;
     break;

   case SNIP:
     if (gesture_ == NONE)
     {
        blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0, emptyKeycode);
        blehid.keyRelease();
        gestureState = IDLE;
     }
     break;

   case ALT_F4:
     if (gesture_ == NONE)
     {
        blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0, emptyKeycode);
        blehid.keyRelease();
        gestureState = IDLE;
     }
     break;

   case ALT_TAB:
     if (gesture_ == NONE)
     {
        blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0, emptyKeycode);
        blehid.keyRelease();
        gestureState = IDLE;
     }
     break;

   case ALT_SHIFT_TAB:
     if (gesture_ == NONE)
     {
        blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0, emptyKeycode);
        blehid.keyRelease();
        gestureState = IDLE;
     }
     break;

   case SCROLL:
    if (gesture_ == NONE)
    {
      mouseEnabled = true;
      blehid.mouseScroll(0);
      keyboardState = KEYBOARD_IDLE;
      gestureState = IDLE;
    }
    break;

   case ZOOM:
    if (gesture_ == NONE)
    {
      blehid.mouseScroll(0);
      blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0, emptyKeycode); // Release Ctrl modifier
      blehid.keyRelease();
      mouseEnabled = true;
      gestureState = IDLE;
    }
    break;
  }

  // Serial.print(_ekf.x[1]); Serial.print(",");
  // Serial.print(_ekf.x[3]); Serial.print(",");
  // Serial.print(gy); Serial.print(",");
  // Serial.print(gz); Serial.print(",");

  //  Serial.print(ax); Serial.print(",");
  // Serial.print(ay); Serial.print(",");
  // Serial.print(az); Serial.print(",");
  // Serial.print(pointer_1);
  // Serial.print(",");
  // Serial.print(middle_1);
  // Serial.print(",");
  // Serial.print(rightclick);
  // Serial.print(",");

  // Serial.print(leftclick);
  // Serial.print(",");
  Serial.print("thumb: ");
  Serial.print(thumb_1); // middle
  // Serial.print(",");

  Serial.print(" pointer: ");
  Serial.print(pointer_1); // pointer
  // Serial.print(",");
  Serial.print(" middle: ");
  Serial.print(middle_1); // thumb

  // Serial.print(",");
  Serial.print(" ring: ");
  Serial.print(ring_1); // ring
                        // Serial.print(",");

  Serial.print(" pinky: ");
  Serial.print(pinky_1); // pinky
  // Serial.print(",");

  Serial.print(curr / 1000.000);
  Serial.println();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);

  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for 'Name' in the advertising packet
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void configBluetooth()
{
  // add any bluetooth config stuff here
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configPrphConn(100, 8, 4, 4);
}
