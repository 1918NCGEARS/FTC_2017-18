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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import static org.firstinspires.ftc.teamcode.AutonModes_v1.State.ALL_STOP;
import static org.firstinspires.ftc.teamcode.AutonModes_v1.State.ARM_DOWN;
import static org.firstinspires.ftc.teamcode.AutonModes_v1.State.ARM_UP;
import static org.firstinspires.ftc.teamcode.AutonModes_v1.State.ID_BALL;
import static org.firstinspires.ftc.teamcode.AutonModes_v1.State.KNOCK_BALL;
import static org.firstinspires.ftc.teamcode.AutonModes_v1.State.LIFT_UP;
import static org.firstinspires.ftc.teamcode.AutonModes_v1.State.MOVE;
import static org.firstinspires.ftc.teamcode.AutonModes_v1.State.RESET_TIME;


/**
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Red1 - Jewel v1", group ="Autonomous")
@Disabled
public class Red1_Jewel_v1 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private DcMotor lift = null;
    private Servo tilt = null;
    private Servo arm = null;

    // Set up tilt servo values
    static final int TILT_MAX_POS_DEG = 1260;     // Maximum rotational position
    static final int TILT_MIN_POS_DEG = 0;     // Minimum rotational position
    static final double TILT_MAX_POS = 1.0;     // Maximum rotational position
    static final double TILT_MIN_POS = 0.0;     // Minimum rotational position
    static final int TILT_DOWN_DEG = 383;
    static final int TILT_CARRY_DEG = 460;  // Originally 475
    static final int TILT_UP_DEG = 533;

    int tiltPosDeg;
    double tiltPos;

    // Set up arm servo values
    static final int ARM_MAX_POS_DEG = 180;
    static final int ARM_MIN_POS_DEG = 0;
    static final double ARM_MAX_POS = 1.0;
    static final double ARM_MIN_POS = 0.0;
//    static final int ARM_DOWN_DEG = 61;     // 6043
    static final int ARM_DOWN_DEG = 55;     // 7911 and 13554
    static final int ARM_UP_DEG = 180;

    int armPosDeg;
    double armPos;

    // Set up color sensors
    private DeviceInterfaceModule dim = null;
    private ColorSensor rgbSensorFront = null;
//    private ColorSensor rgbSensorRear = null;
    private static final int FRONT_LED_CHANNEL = 4;
//    private static final int REAR_LED_CHANNEL = 5;
    private float hsvValuesFront[] = {0F, 0F, 0F};
//    private float hsvValuesRear[] = {0F, 0F, 0F};
    private boolean ledOn = TRUE;

    // Set up encoder values
    static final double PI = 3.14159;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Neverest 40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = .6666666666;     // This is < 1.0 if geared UP     MULT BY .66 FOR NEW SPROCKETS
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);

    int leftDriveEncCount;
    int rightDriveEncCount;
    static final int ENC_ERR_RNG = 50;

    int stateIndex;
    double leftDistTotal;
    double rightDistTotal;
    int knockBallDirection = 0;
    final boolean isBlue = this.getClass().getName().startsWith("B", 31);


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // Set up color sensors
        dim = hardwareMap.deviceInterfaceModule.get("cdim");
        dim.setDigitalChannelMode(FRONT_LED_CHANNEL, DigitalChannel.Mode.OUTPUT);
//        dim.setDigitalChannelMode(REAR_LED_CHANNEL, DigitalChannel.Mode.OUTPUT);
        rgbSensorFront = hardwareMap.colorSensor.get("color_sensor1");
//        rgbSensorRear = hardwareMap.colorSensor.get("color_sensor2");
        dim.setDigitalChannelState(FRONT_LED_CHANNEL, ledOn);
//        dim.setDigitalChannelState(REAR_LED_CHANNEL, ledOn);

        // Set up motors and servos
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        lift = hardwareMap.get(DcMotor.class, "lift");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stateIndex = 0;
        leftDistTotal = 0;
        rightDistTotal = 0;

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
        leftDriveEncCount = 0;
        rightDriveEncCount= 0;
        dim.setDigitalChannelState(FRONT_LED_CHANNEL, ledOn);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Declare local variables
        AutonModes_v1.State state;
        double leftDist = 0;
        double rightDist = 0;
        double leftDrivePower = 0;
        double rightDrivePower = 0;
        double leftIntakePower = 0;
        double rightIntakePower = 0;
        double liftPower = 0;
        int redFront;
//        int redRear;
        int greenFront;
//        int greenRear;
        int blueFront;
//        int blueRear;
        double knockBallVector = 0;
        double hueFront;
//        double hueRear;
        double saturationFront;
//        boolean ballColorKnownRear = FALSE;
        boolean isFrontBallRed = FALSE;
//        boolean isRearBallRed = FALSE;


        // List auton actions and move distances
        AutonModes_v1.State[] autonStatesArr = {LIFT_UP, ARM_DOWN, RESET_TIME, ID_BALL, KNOCK_BALL, RESET_TIME, ARM_UP, MOVE, MOVE, ALL_STOP};

        // Move distances, in inches.  Turns use pivot on one side ==> turn radius of 18 inches
        //                             Circumference = pi * 18^2 = 56.54866776.
        //                             Double to approximately compensate for wheel slip on turns
        double[] leftDriveMoveDist =  {0, 0, 0, 0,  3, 0, 0,  18,      0, 0};
        double[] rightDriveMoveDist = {0, 0, 0, 0,  3, 0, 0,  18, 28.274, 0};

        // Drive motor powers, POSITIVE = BACKWARDS
        // During turns, apply power to both sides so non-turning side can correct if needed
        double[] leftDrivePowerArr =  {0, 0, 0, 0, .3, 0, 0, .75,    .75, 0};
        double[] rightDrivePowerArr = {0, 0, 0, 0, .3, 0, 0, .75,    .75, 0};

//        final int RED_CUTOFF = 25000;
//        final int BLUE_CUTOFF = 16500;
        final double HUE_CUTOFF = 40.0;
        final double SATURATION_CUTOFF = 0.4;

        if (isBlue) {
            leftDriveMoveDist[stateIndex]  = -1 * leftDriveMoveDist[stateIndex];
            rightDriveMoveDist[stateIndex] = -1 * rightDriveMoveDist[stateIndex];
            leftDrivePowerArr[stateIndex]  = -1 * leftDrivePowerArr[stateIndex];
            rightDrivePowerArr[stateIndex] = -1 * rightDrivePowerArr[stateIndex];
        }

        state = autonStatesArr[stateIndex];

        switch (state) {
            case RESET_TIME:
                runtime.reset();
                stateIndex++;
                break;
            case RESET_ENCODERS:
                leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                stateIndex++;
                break;
            case ARM_DOWN:
                armPosDeg -= 4;
                if (armPosDeg <= ARM_DOWN_DEG) {
                    armPosDeg = ARM_DOWN_DEG;
                    stateIndex++;
                }
                break;
            case ARM_UP:
                armPosDeg = ARM_UP_DEG;
                if (runtime.milliseconds() >= 500) {
                    stateIndex++;
                }
                break;
            case MOVE:
                leftDist = leftDistTotal + leftDriveMoveDist[stateIndex];
                rightDist = rightDistTotal + rightDriveMoveDist[stateIndex];

                setEncDrive(leftDist, rightDist);

                if (!leftInRange()) {
                    leftDrivePower = leftDrivePowerArr[stateIndex];
                }
                else {
                    leftDrivePower = 0;
                }

                if (!rightInRange()) {
                    rightDrivePower = rightDrivePowerArr[stateIndex];
                }
                else {
                    rightDrivePower = 0;
                }

                if (leftInRange() && rightInRange()) {
                    leftDistTotal += leftDriveMoveDist[stateIndex];
                    rightDistTotal += rightDriveMoveDist[stateIndex];
                    stateIndex++;
                }
                break;
            case DROP_GLYPH:
                leftDrivePower = leftDrivePowerArr[stateIndex];
                rightDrivePower = rightDrivePowerArr[stateIndex];
                tiltPosDeg = TILT_UP_DEG;
                if (runtime.milliseconds() >= 2000) {
                    stateIndex++;
                }
                break;
            case ALL_STOP:
                leftDrivePower = 0;
                rightDrivePower = 0;
                leftIntakePower = 0;
                rightIntakePower = 0;
                liftPower = 0;
                break;
            case TIME_DELAY:
                if (runtime.milliseconds() >= 5000) {
                    stateIndex++;
                }
                break;
            case ID_BALL:
                redFront = rgbSensorFront.red();
                greenFront = rgbSensorFront.green();
                blueFront = rgbSensorFront.blue();
                Color.RGBToHSV((redFront * 255) / 800, (greenFront * 255) / 800, (blueFront * 255) / 800, hsvValuesFront);
                hueFront = hsvValuesFront[0];
                saturationFront = hsvValuesFront[1];
                telemetry.addData("Front Sensor", "red: %d", redFront);
                telemetry.addData("Front Sensor","green: %d", greenFront);
                telemetry.addData("Front Sensor", "blue: %d", blueFront);
                telemetry.addData("Front Sensor", "hue: %.2f", hueFront);
                telemetry.addData("Front Sensor", "saturation: %.2f", saturationFront);
                telemetry.addData("Front Sensor", "Value: %.2f", hsvValuesFront[2]);

                if (/*(redFront > RED_CUTOFF) && (blueFront < BLUE_CUTOFF) &&*/
                           (hueFront < HUE_CUTOFF) && (saturationFront > SATURATION_CUTOFF)) {
                    telemetry.addData("Front Ball Color", "Red");
                    isFrontBallRed = TRUE;
                } else if (/*(redFront < RED_CUTOFF) && (blueFront > BLUE_CUTOFF) &&*/
                           (hueFront > HUE_CUTOFF) && (saturationFront < SATURATION_CUTOFF)) {
                    telemetry.addData("Front Ball Color", "Blue");
                    isFrontBallRed = FALSE;
                } else {
                    telemetry.addData("Front Ball Color", "Unknown");
                }

                if (runtime.milliseconds() >= 2000) {
                    ledOn = FALSE;
                    if (isFrontBallRed) {
                        knockBallDirection = 1;
                    } else if (!isFrontBallRed) {
                        knockBallDirection = -1;
                    } else {
                        knockBallDirection = 0;
                        stateIndex++;
                    }
                    dim.setDigitalChannelState(FRONT_LED_CHANNEL, ledOn);
                    stateIndex++;
                }
                break;
            case KNOCK_BALL:
                telemetry.addData("knock ball", "direction = %d", knockBallDirection);
                if (knockBallDirection == 0) {
                    stateIndex++;
                } else {
                    telemetry.addData("Knock Ball", "MoveDist: %.2f", leftDriveMoveDist[stateIndex]);
                    knockBallVector = knockBallDirection * leftDriveMoveDist[stateIndex];
                    setEncDrive(knockBallVector, knockBallVector);

                    if (!leftInRange()) {
                        leftDrivePower = leftDrivePowerArr[stateIndex];
                    }
                    else {
                        leftDrivePower = 0;
                    }

                    if (!rightInRange()) {
                        rightDrivePower = rightDrivePowerArr[stateIndex];
                    }
                    else {
                        rightDrivePower = 0;
                    }

                    if (leftInRange() && rightInRange()) {
                        stateIndex++;
                    }
                }
                break;
            case LIFT_UP:
                liftPower = .75;
                if (runtime.milliseconds() >= 250) {
                    liftPower = 0;
                    stateIndex++;
                }
                break;
            default:
                leftDrivePower = 0;
                rightDrivePower = 0;
                leftIntakePower = 0;
                rightIntakePower = 0;
                liftPower = 0;
                telemetry.addData("Status", "state error: %s", state);
        }

        // Map servo values
        tiltPos = map(tiltPosDeg, TILT_MIN_POS_DEG, TILT_MAX_POS_DEG, TILT_MIN_POS, TILT_MAX_POS);
        armPos = map(armPosDeg, ARM_MIN_POS_DEG, ARM_MAX_POS_DEG, ARM_MIN_POS, ARM_MAX_POS);

        // Set powers for motors and positions for servos
        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);
        lift.setPower(liftPower);
        tilt.setPosition(tiltPos);
        arm.setPosition(armPos);

        // Send telemetry data
        telemetry.addData("State", "(%s)", state);
        telemetry.addData("Drive", "left: (%.2f), right: (%.2f)", leftDrivePower, rightDrivePower);
        telemetry.addData("Intake", "left: (%.2f), right: (%.2f)", leftIntakePower, rightIntakePower);
        telemetry.addData("Tilt Angle", "Degrees: %d    Pos Value: %.2f", tiltPosDeg, tiltPos);
        telemetry.addData("Arm Angle", "Degrees: %d    Pos Value: %.2f", armPosDeg, armPos);
    }

    boolean leftInRange() {
        if ((leftDrive.getCurrentPosition() >= leftDriveEncCount - ENC_ERR_RNG) &&
            (leftDrive.getCurrentPosition() <= leftDriveEncCount + ENC_ERR_RNG)) {
            return TRUE;
        } else {
            return FALSE;
        }
    }

    boolean rightInRange() {
        if ((rightDrive.getCurrentPosition() >= rightDriveEncCount - ENC_ERR_RNG) &&
            (rightDrive.getCurrentPosition() <= rightDriveEncCount + ENC_ERR_RNG)) {
            return TRUE;
        } else {
            return FALSE;
        }
    }

    double map(int input, int inMin, int inMax, double outMin, double outMax) {
        double output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        return output;
    }

    void setEncDrive(double leftDriveDist, double rightDriveDist) {
        leftDriveEncCount = (int) Math.round(leftDriveDist * COUNTS_PER_INCH);
        rightDriveEncCount = (int) Math.round(rightDriveDist * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(leftDriveEncCount);
        rightDrive.setTargetPosition(rightDriveEncCount);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}