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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * INCREMENT sets how much to increase/decrease the tilt position each cycle
 * CYCLE_MS sets the update period.
 *
 * NOTE: When any tilt position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 */
@TeleOp(name = "Servo Position Test - iterative", group = "Iterative OpMode")
@Disabled
public class ServoPositionTest_Iterative extends OpMode {

//    static final double INCREMENT   = 0.01;     // amount to slew tilt each CYCLE_MS cycle
//    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final int ARM_MAX_POS_DEG =  180;     // Maximum rotational position
    static final int ARM_MIN_POS_DEG =  0;     // Minimum rotational position
    static final double ARM_MAX_POS =  1.0;     // Maximum rotational position
    static final double ARM_MIN_POS =  0.0;     // Minimum rotational position

    // Define class members
    Servo arm = null;
//    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
//    boolean rampUp = true;
    int armPort;
    int armPosDeg;
    double armPos;


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        arm = hardwareMap.get(Servo.class, "arm");
        armPort = arm.getPortNumber();
        telemetry.addData("Arm Servo Channel","(%d)", armPort);
        telemetry.addData("Status","Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        armPosDeg = 90;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.a) {
//            armPosDeg = 0;
            if (armPosDeg > 0) {
                armPosDeg = armPosDeg - 1;
            }
        }
//        else if (gamepad1.x) {
//            armPosDeg = 30;
//        }
        else if (gamepad1.y){
//            armPosDeg = 100;
            if (armPosDeg < 180) {
                armPosDeg = armPosDeg + 1;
            }
        }

        armPos = (armPosDeg - ARM_MIN_POS_DEG) * (ARM_MAX_POS - ARM_MIN_POS) /
                  (ARM_MAX_POS_DEG - ARM_MIN_POS_DEG) + ARM_MIN_POS;
        arm.setPosition(armPos);

        telemetry.addData("Position", "Degrees: %d", armPosDeg);
        telemetry.addData("Position", "%.2f", armPos);
    }
    /*@Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        tilt = hardwareMap.get(Servo.class, "tilt");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan tilt till stop pressed.
        while(opModeIsActive()){

            // slew the tilt, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the tilt to the new position and pause;
            tilt.setPosition(position);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }*/
}
