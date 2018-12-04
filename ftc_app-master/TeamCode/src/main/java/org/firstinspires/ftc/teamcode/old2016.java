/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="old2016", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class old2016 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    static final double LIFT_HEIGHT = 1000;
    static final double ERROR_RANGE = 50;
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;
    private DcMotor towerMotor = null;
    private DcMotor towerMotor2 = null;
    private TouchSensor limitSwitch = null;
    private Servo collectorServo = null;
    private Servo shooterServo = null;
    private final double SHOOTER_START = .54;
    private final double SHOOTER_END = .85;
    //private boolean limit = limitSwitch.isPressed();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftFrontMotor = hardwareMap.dcMotor.get("left motor 1");
        rightFrontMotor = hardwareMap.dcMotor.get("right motor 1");
        leftRearMotor = hardwareMap.dcMotor.get("left motor");
        rightRearMotor = hardwareMap.dcMotor.get("right motor");
        towerMotor = hardwareMap.dcMotor.get("tower motor");
        towerMotor2 = hardwareMap.dcMotor.get("tower motor 1");
        collectorServo = hardwareMap.servo.get("collector tilt");
        shooterServo = hardwareMap.servo.get("shooter tilt");
        limitSwitch = hardwareMap.touchSensor.get("limit switch");

        //leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //towerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //towerMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        towerMotor.setDirection(DcMotor.Direction.REVERSE);
        towerMotor2.setDirection(DcMotor.Direction.REVERSE);

        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        towerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        towerMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        collectorServo.setPosition(.5);
        shooterServo.setPosition(SHOOTER_END);
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
        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        shooterServo.setPosition(SHOOTER_START);
        leftFrontMotor.setPower(-gamepad1.left_stick_y);
        rightFrontMotor.setPower(-gamepad1.right_stick_y);
        leftRearMotor.setPower(-gamepad1.left_stick_y);
        rightRearMotor.setPower(-gamepad1.right_stick_y);
        /*if (gamepad1.left_bumper) {
            servoPosition = 0;
        }
        if (gamepad1.right_bumper) {
            servoPosition = 1;
        }*/
        //positionDegrees = position / 180;

        if (gamepad1.a){
            if (towerMotor2.getCurrentPosition() < LIFT_HEIGHT - ERROR_RANGE) {
                towerMotor2.setPower(.7);
            }
            else if (towerMotor2.getCurrentPosition() > LIFT_HEIGHT + ERROR_RANGE) {
                towerMotor2.setPower(-.7);
            }
            else
                towerMotor2.setPower(0);
        }
        else {
            if (limitSwitch.isPressed()) {
                if (-gamepad2.left_stick_y > 0) {
                    towerMotor2.setPower(0);
                } else {
                    towerMotor2.setPower(-gamepad2.left_stick_y);
                }
            } else {
                towerMotor2.setPower(-gamepad2.left_stick_y);
            }
        }
        towerMotor.setPower(towerMotor2.getPower());

        if (gamepad2.right_bumper) {
            collectorServo.setPosition(1);
        }
        else if (gamepad2.left_bumper) {
            collectorServo.setPosition(0);
        }
        else {
            collectorServo.setPosition(.5);
        }

        telemetry.addData("limit switch",limitSwitch);
        telemetry.addData("right bumper",gamepad1.right_bumper);
        telemetry.addData("left bumper",gamepad1.left_bumper);
        telemetry.addData("left encoder",leftRearMotor.getCurrentPosition());
        telemetry.addData("right encoder",rightRearMotor.getCurrentPosition());
        telemetry.addData("tower encoder 2",towerMotor2.getCurrentPosition());


        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("left stick","%.2f",-gamepad1.left_stick_y);
        telemetry.addData("right stick","%.2f",-gamepad1.right_stick_y);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        towerMotor.setPower(0);
        towerMotor2.setPower(0);
    }

}
