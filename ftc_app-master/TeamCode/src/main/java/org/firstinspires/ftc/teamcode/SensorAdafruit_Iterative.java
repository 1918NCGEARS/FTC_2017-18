/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Boolean.FALSE;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * the Adafruit RGB Sensor.  It assumes that the I2C
 * cable for the sensor is connected to an I2C port on the
 * Core Device Interface Module.
 *
 * It also assuems that the LED pin of the sensor is connected
 * to the digital signal pin of a digital port on the
 * Core Device Interface Module.
 *
 * You can use the digital port to turn the sensor's onboard
 * LED on or off.
 *
 * The op mode assumes that the Core Device Interface Module
 * is configured with a name of "dim" and that the Adafruit color sensor
 * is configured as an I2C device with a name of "sensor_color".
 *
 * It also assumes that the LED pin of the RGB sensor
 * is connected to the signal pin of digital port #5 (zero indexed)
 * of the Core Device Interface Module.
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Color Sensor - Iterative", group = "Iterative Opmode")
@Disabled                            // Comment this out to add to the opmode list
public class SensorAdafruit_Iterative extends OpMode {

  private ColorSensor rgbSensor = null;
  private DeviceInterfaceModule dim = null;
  private ElapsedTime runtime = new ElapsedTime();

  // we assume that the LED pin of the RGB sensor is connected to
  // digital port 5 (zero indexed).
  private static final int LED_CHANNEL = 5;

  // bPrevState and bCurrState represent the previous and current state of the button.
  boolean bPrevState;
  boolean bCurrState;

  // hsvValues is an array that will hold the hue, saturation, and value information.
  float hsvValues[] = {0F, 0F, 0F};

  // ledOn represents the state of the LED.
  boolean ledOn = true;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initializing...");

    bPrevState = FALSE;
    bCurrState = FALSE;

    // get a reference to our DeviceInterfaceModule object.
    dim = hardwareMap.deviceInterfaceModule.get("cdim");

    // set the digital channel to output mode.
    // remember, the Adafruit sensor is actually two devices.
    // It's an I2C sensor and it's also an LED that can be turned on or off.
    dim.setDigitalChannelMode(LED_CHANNEL, DigitalChannel.Mode.OUTPUT);

    // get a reference to our ColorSensor object.
    rgbSensor = hardwareMap.colorSensor.get("color_sensor");

    // turn the LED on in the beginning, just so user will know that the sensor is active.
    dim.setDigitalChannelState(LED_CHANNEL, ledOn);

    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    telemetry.addData("Status", "init_loop");
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {

    // check the status of the x button on gamepad.
    bCurrState = gamepad1.x;

    // check for button-press state transitions.
    if ((bCurrState) && (bCurrState != bPrevState))  {
      // button is transitioning to a pressed state. Toggle the LED.
      ledOn = !ledOn;
      dim.setDigitalChannelState(LED_CHANNEL, ledOn);
    }

    // update previous state variable.
    bPrevState = bCurrState;

    // convert the RGB values to HSV values.
    Color.RGBToHSV((rgbSensor.red() * 255) / 800, (rgbSensor.green() * 255) / 800, (rgbSensor.blue() * 255) / 800, hsvValues);

    // send the info back to driver station using telemetry function.
    telemetry.addData("LED", ledOn ? "On" : "Off");
    telemetry.addData("Clear", rgbSensor.alpha());
    telemetry.addData("Red  ", rgbSensor.red());
    telemetry.addData("Green", rgbSensor.green());
    telemetry.addData("Blue ", rgbSensor.blue());
    telemetry.addData("Hue", hsvValues[0]);
    telemetry.update();
  }
}
