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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Summer Camp", group="Iterative Opmode")
//@Disabled
public class SummerCamp2018 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private DcMotor lift = null;

    private DeviceInterfaceModule dim = null;
    private DigitalChannel lowerLimit = null;
    private DigitalChannel upperLimit = null;

    private static final int LOWER_LIMIT_CHANNEL = 0;
    private static final int UPPER_LIMIT_CHANNEL = 1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        lift = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        dim = hardwareMap.deviceInterfaceModule.get("cdim");
        lowerLimit = hardwareMap.digitalChannel.get("lower_limit");
        upperLimit = hardwareMap.digitalChannel.get("upper_limit");
        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);
//        dim.setDigitalChannelMode(LOWER_LIMIT_CHANNEL, DigitalChannel.Mode.INPUT);
//        dim.setDigitalChannelMode(UPPER_LIMIT_CHANNEL, DigitalChannel.Mode.INPUT);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

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
        // Setup a variable for each motor and servo
        double leftDrivePower;
        double rightDrivePower;
        double intakePower;
        double liftPower;
        boolean lowerLimitPressed;
        boolean upperLimitPressed;

        leftDrivePower = gamepad1.left_stick_y;
        rightDrivePower = gamepad1.right_stick_y;

        intakePower = gamepad2.left_stick_y * .6;

        lowerLimitPressed = lowerLimit.getState();
        upperLimitPressed = upperLimit.getState();

        if (gamepad2.y && !upperLimitPressed) {
            liftPower = .2;
        }
        else if (gamepad2.a && !lowerLimitPressed) {
            liftPower = -.2;
        }
        else{
            liftPower = 0;
        }

        // Send motor powers and servo positions
        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
        lift.setPower(liftPower);

        // Send data to driver station
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive", "left (%.2f), right (%.2f)", leftDrivePower, rightDrivePower);
        telemetry.addData("Intake", "(%.2f)", intakePower);
        telemetry.addData("Lift", "tilt (%.2f)", liftPower);
        telemetry.addData("Limit", "lower: %b", lowerLimitPressed);
        telemetry.addData("Limit", "upper: %b", upperLimitPressed);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}
