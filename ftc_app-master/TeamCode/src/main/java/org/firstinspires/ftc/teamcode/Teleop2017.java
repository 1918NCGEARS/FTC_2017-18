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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2017 Teleop", group="Iterative Opmode")
@Disabled
public class Teleop2017 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private DcMotor lift = null;
    private Servo tilt = null;
    private Servo arm = null;

    // Set up tilt servo values
    static final int TILT_MAX_POS_DEG     =  1260;     // Maximum rotational position
    static final int TILT_MIN_POS_DEG     =  0;     // Minimum rotational position
    static final double TILT_MAX_POS     =  1.0;     // Maximum rotational position
    static final double TILT_MIN_POS     =  0.0;     // Minimum rotational position
    static final int TILT_DOWN_DEG = 383;
    static final int TILT_CARRY_DEG = 460;  // Originally 475
    static final int TILT_UP_DEG = 533;

    int tiltPosDeg;
    double tiltPos;

    // Set up arm servo values
    static final int ARM_MAX_POS_DEG = 185;
    static final int ARM_MIN_POS_DEG = 0;
    static final double ARM_MAX_POS = 1.0;
    static final double ARM_MIN_POS = 0.0;
    static final int ARM_DOWN_DEG = 63;
    static final int ARM_UP_DEG = 185;

    int armPosDeg;
    double armPos;

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
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Initialize servos
        tiltPosDeg = TILT_CARRY_DEG;
        tiltPos = map(tiltPosDeg, TILT_MIN_POS_DEG, TILT_MAX_POS_DEG, TILT_MIN_POS, TILT_MAX_POS);
        tilt.setPosition(tiltPos);

        armPosDeg = ARM_UP_DEG;
        armPos = map(armPosDeg, ARM_MIN_POS_DEG, ARM_MAX_POS_DEG, ARM_MIN_POS, ARM_MAX_POS);
        arm.setPosition(armPos);
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
        // Setup a variable for each motor and servo
        double leftDrivePower;
        double rightDrivePower;
        double leftIntakePower;
        double rightIntakePower;
        double liftPower;

        leftDrivePower = gamepad1.left_stick_y;
        rightDrivePower = gamepad1.right_stick_y;

        leftIntakePower = gamepad2.left_stick_y * .3;
        rightIntakePower = gamepad2.right_stick_y * .3;

        if (gamepad1.a) {
            armPosDeg = ARM_UP_DEG;
        }

        if (gamepad2.y) {
            tiltPosDeg = TILT_CARRY_DEG;
            liftPower = 1.0;
        }
        else if (gamepad2.a) {
            liftPower = -1.0;
        }
        else{
            liftPower = 0;
        }

        if (gamepad2.right_bumper || gamepad1.right_bumper) {
            tiltPosDeg = TILT_CARRY_DEG;
        }
        else if (gamepad2.left_bumper || gamepad1.left_bumper) {
            tiltPosDeg = TILT_UP_DEG;
        }
        else if ((gamepad2.left_trigger > 0) || (gamepad1.left_trigger > 0)) {
            tiltPosDeg = TILT_DOWN_DEG;
        }

        tiltPos = (tiltPosDeg - TILT_MIN_POS_DEG) * (TILT_MAX_POS - TILT_MIN_POS) /
                  (TILT_MAX_POS_DEG - TILT_MIN_POS_DEG) + TILT_MIN_POS;
        armPos = (armPosDeg - ARM_MIN_POS_DEG) * (ARM_MAX_POS - ARM_MIN_POS) /
                (ARM_MAX_POS_DEG - ARM_MIN_POS_DEG) + ARM_MIN_POS;

        // Send motor powers and servo positions
        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);
        lift.setPower(liftPower);
        tilt.setPosition(tiltPos);
        arm.setPosition(armPos);

        // Send data to driver station
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive", "left (%.2f), right (%.2f)", leftDrivePower, rightDrivePower);
        telemetry.addData("Intake", "left (%.2f), right (%.2f)", leftIntakePower, rightIntakePower);
        telemetry.addData("Lift", "tilt (%.2f)", liftPower);
        telemetry.addData("Tilt Angle", "Degrees: %d    Pos Value: %.2f", tiltPosDeg, tiltPos);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    double map(int input, int inMin, int inMax, double outMin, double outMax) {
        double output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        return output;
    }
}
