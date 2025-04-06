/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*********************************************************************
MIT License

Copyright (c) 2023, 2024 touchgadgetdev@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*********************************************************************/

/* This program converts a Logitech flight joystick to a Nintendo
 * Switch compatible gamepad. This allows a player to play two joystick
 * games using one hand. Some code is taken from an Adafruit example
 * program so Adafruit's copyright is included.
 *
 * - Device run on the native usb controller with type C USB connector.
 * - Host run on bit-banging 2 GPIOs with the help of Pico-PIO-USB library
 *   with type A USB connector.
 *
 * Requirements:
 * - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
 * - 2 consecutive GPIOs: D+ is defined by PIN_PIO_USB_HOST_DP, D- = D+ +1
 * - Provide VBus (5v) and GND for peripheral
 * - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
 */

// Set this to 0 for use with a Nintendo Switch.
#define USB_DEBUG 0

#if USB_DEBUG
#define DBG_print(...)    Serial.print(__VA_ARGS__)
#define DBG_println(...)  Serial.println(__VA_ARGS__)
#define DBG_printf(...)   Serial.printf(__VA_ARGS__)
#else
#define DBG_print(...)
#define DBG_println(...)
#define DBG_printf(...)
#endif

#define DPAD_NORTH  (0)
#define DPAD_NE     (1)
#define DPAD_EAST   (2)
#define DPAD_SE     (3)
#define DPAD_SOUTH  (4)
#define DPAD_SW     (5)
#define DPAD_WEST   (6)
#define DPAD_NW     (7)

typedef struct {
  uint8_t x;
  uint8_t y;
} DPAD_COORDS_t;

const uint8_t AXIS_MIN = 0;
const uint8_t AXIS_MID = 127;
const uint8_t AXIS_MAX = 255;

#define DPAD_MAX (8)
const DPAD_COORDS_t DPAD_TABLE[DPAD_MAX] = {
  {AXIS_MID, AXIS_MIN}, // DPAD NORTH
  {AXIS_MAX, AXIS_MIN}, // DPAD NORTH EAST
  {AXIS_MAX, AXIS_MID}, // DPAD EAST
  {AXIS_MAX, AXIS_MAX}, // DPAD SOUTH EAST
  {AXIS_MID, AXIS_MAX}, // DPAD SOUTH
  {AXIS_MIN, AXIS_MAX}, // DPAD SOUTH WEST
  {AXIS_MIN, AXIS_MID}, // DPAD WEST
  {AXIS_MIN, AXIS_MIN}, // DPAD NORTH WEST
};

uint8_t DPAD_Y(uint8_t hat) {
  if (hat < DPAD_MAX) {
    return DPAD_TABLE[hat].y;
  }
  return AXIS_MID;
}

uint8_t DPAD_X(uint8_t hat) {
  if (hat < DPAD_MAX) {
    return DPAD_TABLE[hat].x;
  }
  return AXIS_MID;
}

// Kensington Eagle Trackball report layout
// Trackball X, Y axes
// 4 buttons
// Scroll wheel
typedef struct __attribute__ ((packed)) {
  uint32_t x : 10;      // 0..512..1023
  uint32_t y : 10;      // 0..512..1023
  uint8_t buttons;      // 4 buttons
  int8_t scroll;        // Scroll wheel
} Kensington_Eagle_t ;

// Kensington Eagle Trackball state
typedef struct {
  Kensington_Eagle_t report;
  const uint16_t USB_VID = 0x047d;
  const uint16_t USB_PID = 0x2068;
  const Axis_Control_t X_Control = { 8, 0, 511, 1023 };
  const Axis_Control_t Y_Control = { 8, 0, 511, 1023 };
  const Axis_Control_t Scroll_Control = { 4, -127, 0, 127 };
  uint8_t dev_addr;
  uint8_t instance;
  uint8_t report_len;
  bool connected = false;
  bool available = false;
  bool debug = false;
} Kensington_Eagle_state_t;

volatile Kensington_Eagle_state_t Keagle;

// GPIO Pins for Pushbuttons
#define BUTTON_A_PIN 4
#define BUTTON_B_PIN 5
#define BUTTON_X_PIN 6
#define BUTTON_Y_PIN 9
#define BUTTON_UP_PIN 10
#define BUTTON_DOWN_PIN 11
#define BUTTON_LEFT_PIN 12
#define BUTTON_RIGHT_PIN 13

// pio-usb is required for rp2040 usb host
#include "pio_usb.h"
#include "pio-usb-host-pins.h"
#include "Adafruit_TinyUSB.h"
#include "switch_tinyusb.h"

// USB Device gamepad
Adafruit_USBD_HID G_usb_hid;
NSGamepad Gamepad(&G_usb_hid);

// USB Host object for Kensington Trackball
Adafruit_USBH_Host USBHost;

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup()
{
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif
#if USB_DEBUG
  Serial.begin(115200);
#else
  Serial.end();     // Remove CDC ACM port
#endif
  TinyUSBDevice.setID(0x047d, 0x2068);
  Gamepad.begin();

  // Initialize GPIO pins
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_X_PIN, INPUT_PULLUP);
  pinMode(BUTTON_Y_PIN, INPUT_PULLUP);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif

  DBG_println("Switch TinyUSB Gamepad mounted");
}

void print_KEagle_controls()
{
  DBG_printf("X:%d,Y:%d,", Keagle.report.x, Keagle.report.y);
  DBG_printf("scroll:%d,", Keagle.report.scroll);
  DBG_printf("buttons:0x%x", Keagle.report.buttons);
  DBG_println();
}

// Function to print joystick positions and button states for debugging
void print_debug_info(int left_x, int left_y, int right_x, int right_y, uint16_t buttons) {
  DBG_printf("Left Joystick - X: %d, Y: %d\n", left_x, left_y);
  DBG_printf("Right Joystick - X: %d, Y: %d\n", right_x, right_y);
  DBG_printf("Buttons State: 0x%04x\n", buttons);
}

// Sprint subroutine
void sprint() {
  int sprint_iterations = 5; // Number of iterations for sprint
  int sprint_delay = 50;     // Delay in milliseconds between each iteration

  for (int i = 0; i < sprint_iterations; i++) {
    // Push one joystick forward and the other backward
    Gamepad.leftYAxis(0);
    Gamepad.rightYAxis(255);
    Gamepad.loop();
    delay(sprint_delay);

    // Reverse their positions
    Gamepad.leftYAxis(255);
    Gamepad.rightYAxis(0);
    Gamepad.loop();
    delay(sprint_delay);
  }

  // Set both joysticks full forward
  Gamepad.leftYAxis(0);
  Gamepad.rightYAxis(0);
  Gamepad.loop();
}

void loop() {
  if (Keagle.connected) {
    if (Keagle.available) {
      if (sizeof(Keagle.report) == Keagle.report_len) {
        if (Keagle.debug) {
          uint8_t *rpt = (uint8_t *)&Keagle.report;
          DBG_printf("KEagle report(%d): ", Keagle.report_len);
          for (uint16_t i = 0; i < Keagle.report_len; i++) {
            DBG_printf("0x%02X ", rpt[i]);
          }
          DBG_println();
        }
        // Remote wakeup
        if ( TinyUSBDevice.suspended() ) {
          // Wake up host if we are in suspend mode
          // and REMOTE_WAKEUP feature is enabled by host
          TinyUSBDevice.remoteWakeup();
        }
        if (Gamepad.ready()) {
          volatile Kensington_Eagle_t *rpt = &Keagle.report;
          static uint16_t old_buttons = 0;
          uint16_t buttons = rpt->buttons;
          int left_x_joystick = map(rpt->x, Keagle.X_Control.MIN,
              Keagle.X_Control.MAX, 0, 255);
          int left_y_joystick = map(rpt->y, Keagle.Y_Control.MIN,
              Keagle.Y_Control.MAX, 0, 255);
          int right_x_joystick = rpt->scroll;
          int right_y_joystick = DPAD_Y(rpt->hat);

          // Calculate the positions for tank steer
          int left_joystick = left_y_joystick + right_x_joystick;
          int right_joystick = left_y_joystick - right_x_joystick;

          // Constrain the joystick values
          left_joystick = constrain(left_joystick, 0, 255);
          right_joystick = constrain(right_joystick, 0, 255);

          if ((abs(left_x_joystick - 127) > Keagle.X_Control.DEADZONE) ||
              (abs(left_y_joystick - 127) > Keagle.Y_Control.DEADZONE) ||
              (abs(right_x_joystick - 127) > Keagle.X_Control.DEADZONE) ||
              (abs(right_y_joystick - 127) > Keagle.Y_Control.DEADZONE) ||
              (buttons != old_buttons)) {
            if (Keagle.debug) {
              DBG_printf("x=%4d, y=%4d, scroll=%3d, buttons=0x%02x\r\n",
                  rpt->x, rpt->y, rpt->scroll,
                  rpt->buttons);
              DBG_printf("left_x=%6d, left_y=%6d, right_x=%6d, right_y=%6d\r\n",
                  left_x_joystick, left_y_joystick,
                  right_x_joystick, right_y_joystick);
            }
            Gamepad.leftXAxis(left_joystick);
            Gamepad.leftYAxis(left_y_joystick);
            Gamepad.rightXAxis(right_joystick);
            Gamepad.rightYAxis(right_y_joystick);
            Gamepad.buttons(buttons);
            Gamepad.loop();
            old_buttons = buttons;

            // Print debugging information
            print_debug_info(left_joystick, left_y_joystick, right_joystick, right_y_joystick, buttons);
          }
        }
      }
      Keagle.available = false;
    }
  }

  // Read GPIO button states and map to gamepad buttons
  if (digitalRead(BUTTON_A_PIN) == LOW) {
    Gamepad.pressButton(NSGAMEPAD_BUTTON_A);
  } else {
    Gamepad.releaseButton(NSGAMEPAD_BUTTON_A);
  }
  
  if (digitalRead(BUTTON_B_PIN) == LOW) {
    Gamepad.pressButton(NSGAMEPAD_BUTTON_B);
  } else {
    Gamepad.releaseButton(NSGAMEPAD_BUTTON_B);
  }
  
  if (digitalRead(BUTTON_X_PIN) == LOW) {
    Gamepad.pressButton(NSGAMEPAD_BUTTON_X);
  } else {
    Gamepad.releaseButton(NSGAMEPAD_BUTTON_X);
  }
  
  if (digitalRead(BUTTON_Y_PIN) == LOW) {
    Gamepad.pressButton(NSGAMEPAD_BUTTON_Y);
  } else {
    Gamepad.releaseButton(NSGAMEPAD_BUTTON_Y);
  }
  
  if (digitalRead(BUTTON_UP_PIN) == LOW) {
    Gamepad.hatUp();
  } else if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
    Gamepad.hatDown();
  } else if (digitalRead(BUTTON_LEFT_PIN) == LOW) {
    Gamepad.hatLeft();
  } else if (digitalRead(BUTTON_RIGHT_PIN) == LOW) {
    Gamepad.hatRight();
  } else {
    Gamepad.hatCenter();
  }
}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

void setup1() {
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif
  DBG_println("Core1 setup to run TinyUSB host with pio-usb");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
#if USB_DEBUG
    while (!Serial) { delay(1); }
#endif
    DBG_printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    DBG_println("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed");
    while(1) delay(1);
  }

#ifdef PIN_PIO_USB_HOST_VBUSEN
  pinMode(PIN_PIO_USB_HOST_VBUSEN, OUTPUT);
  digitalWrite(PIN_PIO_USB_HOST_VBUSEN, PIN_PIO_USB_HOST_VBUSEN_STATE);
#endif

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_PIO_USB_HOST_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

// core1's loop
void loop1() {
  USBHost.task();
}

extern "C" {

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use.
// tuh_hid_parse_report_descriptor() can be used to parse common/simple enough
// descriptor. Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE,
// it will be skipped therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  DBG_printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  DBG_printf("VID = %04x, PID = %04x\r\n", vid, pid);
  if ((vid == Keagle.USB_VID) && (pid == Keagle.USB_PID)) {
    DBG_printf("Kensington Eagle Trackball (%d) connected\r\n", instance);
    Keagle.connected = true;
    Keagle.available = false;
    Keagle.dev_addr = dev_addr;
    Keagle.instance = instance;
    memset((Kensington_Eagle_t *)&Keagle.report, 0, sizeof(Keagle.report));
  }
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  DBG_printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  if ((Keagle.dev_addr == dev_addr) && (Keagle.instance == instance)) {
    if (Keagle.connected) {
      Keagle.connected = false;
      Keagle.available = false;
      DBG_printf("Kensington Eagle Trackball (%d) disconnected\r\n", instance);
    }
  }
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  if (Keagle.connected && (Keagle.dev_addr == dev_addr) && (Keagle.instance == instance)) {
    memcpy((Kensington_Eagle_t *)&Keagle.report, report, min(sizeof(Keagle.report), len));
    Keagle.report_len = len;
    Keagle.available = true;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

} // extern "C"
