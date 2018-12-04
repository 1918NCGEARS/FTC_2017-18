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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

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
@TeleOp(name = "Ball Color Test", group = "Iterative Opmode")
@Disabled                            // Comment this out to add to the opmode list
public class Ball_Color_Test extends OpMode {

  private ColorSensor rgbSensorFront = null;
  private DeviceInterfaceModule dim = null;
  Servo arm = null;
  private ElapsedTime runtime = new ElapsedTime();

  // we assume that the LED pin of the RGB sensor is connected to
  // digital port 5 (zero indexed).
  private static final int FRONT_LED_CHANNEL = 4;

  // hsvValues is an array that will hold the hue, saturation, and value information.
  float hsvValues[] = {0F, 0F, 0F};

  // ledOn represents the state of the LED.
  boolean ledOn = true;

  // Set up arm servo values
  static final int ARM_MAX_POS_DEG = 180;
  static final int ARM_MIN_POS_DEG = 0;
  static final double ARM_MAX_POS = 1.0;
  static final double ARM_MIN_POS = 0.0;
  static final int ARM_DOWN_DEG = 61;
  static final int ARM_UP_DEG = 180;

  int armPosDeg;
  double armPos;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initializing...");

    dim = hardwareMap.deviceInterfaceModule.get("cdim");
    dim.setDigitalChannelMode(FRONT_LED_CHANNEL, DigitalChannel.Mode.OUTPUT);
    rgbSensorFront = hardwareMap.colorSensor.get("color_sensor1");
    dim.setDigitalChannelState(FRONT_LED_CHANNEL, ledOn);

    arm = hardwareMap.get(Servo.class, "arm");
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
    dim.setDigitalChannelState(FRONT_LED_CHANNEL, ledOn);
    armPosDeg = ARM_UP_DEG;
    armPos = map(armPosDeg, ARM_MIN_POS_DEG, ARM_MAX_POS_DEG, ARM_MIN_POS, ARM_MAX_POS);
    arm.setPosition(armPos);
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    int redFront;
    int greenFront;
    int blueFront;
    double hueFront;
    double saturationFront;
    double valueFront;

    final int RED_CUTOFF = 25000;
    final int BLUE_CUTOFF = 16500;
    final double HUE_CUTOFF = 50.0;
    final double SATURATION_CUTOFF = 0.4;

    armPosDeg = ARM_DOWN_DEG;
    armPos = map(armPosDeg, ARM_MIN_POS_DEG, ARM_MAX_POS_DEG, ARM_MIN_POS, ARM_MAX_POS);
    arm.setPosition(armPos);

    redFront = rgbSensorFront.red();
    greenFront = rgbSensorFront.green();
    blueFront = rgbSensorFront.blue();

    // convert the RGB values to HSV values.
    Color.RGBToHSV((redFront * 255) / 800, (greenFront * 255) / 800, (blueFront * 255) / 800, hsvValues);
    hueFront = hsvValues[0];
    saturationFront = hsvValues[1];
    valueFront = hsvValues[2];

    if ((redFront > RED_CUTOFF) && (blueFront < BLUE_CUTOFF) &&
            (hueFront < HUE_CUTOFF) && (saturationFront > SATURATION_CUTOFF)) {
      telemetry.addData("Front Ball Color", "Red");
    } else if ((redFront < RED_CUTOFF) && (blueFront > BLUE_CUTOFF) &&
            (hueFront > HUE_CUTOFF) && (saturationFront < SATURATION_CUTOFF)) {
      telemetry.addData("Front Ball Color", "Blue");
    } else {
      telemetry.addData("Front Ball Color", "Unknown");
    }

    // send the info back to driver station using telemetry function.
    telemetry.addData("Front Sensor", "red: %d", redFront);
    telemetry.addData("Front Sensor","green: %d", greenFront);
    telemetry.addData("Front Sensor", "blue: %d", blueFront);
    telemetry.addData("Front Sensor", "hue: %.2f", hueFront);
    telemetry.addData("Front Sensor", "saturation: %.2f", saturationFront);
    telemetry.addData("Front Sensor", "Value: %.2f", valueFront);
  }

  double map(int input, int inMin, int inMax, double outMin, double outMax) {
    double output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return output;
  }
}
