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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.ALL_STOP;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.ARM_DOWN;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.ARM_UP;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.DROP_GLYPH;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.ID_BALL;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.ID_PICTO;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.KNOCK_BALL;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.MOVE;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.RESET_ENCODERS;
import static org.firstinspires.ftc.teamcode.Auton_v1_2Color_4Drive.State.RESET_TIME;

/**
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Blue1 - 2 Color, 4 Drive", group ="Autonomous")
@Disabled
public class Auton_v1_2Color_4Drive extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Set up Vuforia
    private VuforiaLocalizer vuforia;
    //    int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    // Set up color sensors
    private DeviceInterfaceModule dim = null;
    private ColorSensor rgbSensorFront = null;
    private ColorSensor rgbSensorRear = null;
    private static final int FRONT_LED_CHANNEL = 4;
    private static final int REAR_LED_CHANNEL = 5;
    private float hsvValuesFront[] = {0F, 0F, 0F};
    private float hsvValuesRear[] = {0F, 0F, 0F};
    private boolean ledOn = TRUE;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
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
    static final int TILT_DOWN_DEG = 0;
    static final int TILT_CARRY_DEG = 30;
    static final int TILT_UP_DEG = 100;
    int tiltPosDeg;
    double tiltPos;

    // Set up arm servo values
    static final int ARM_MAX_POS_DEG = 1260;     // Maximum rotational position
    static final int ARM_MIN_POS_DEG = 0;     // Minimum rotational position
    static final double ARM_MAX_POS = 1.0;     // Maximum rotational position
    static final double ARM_MIN_POS = 0.0;     // Minimum rotational position
    static final int ARM_DOWN_DEG = 0;
    static final int ARM_UP_DEG = 90;
    int armPosDeg;
    double armPos;

    // Set up encoder values
    static final double PI = 3.14159;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Neverest 40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);

    int leftDriveEncCount;
    int rightDriveEncCount;
    static final int ENC_ERR_RNG = 50;

    // Create auton enum list
    enum State {
        ID_PICTO, ID_BALL, KNOCK_BALL, ARM_UP, ARM_DOWN, MOVE, RESET_TIME, RESET_ENCODERS, DROP_GLYPH, ALL_STOP
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // Map hardware and set motor direction
        // Set up Vuforia and camera
        parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AeO63Ib/////AAAAGb/HhQ6TaU6ts13SmLvsIrhSY1hyodBnppjIhUNM/w6us3/wpkIPS181MPnmIuoaMx0s+YsRuZdEaSXn8MOah0ip17sNAZ0L3bDefahS8v1g1HCrcrzIFPZcQfGcaYU21qf05Z0Rp1v6iHIR2j9NFhfVklxdFz/PIe1G3PY2oKW15N1i+2FqMCapNrKyMO75n8WCTcT4YcpCiBm/afNqovKbmPeaCnFcI7dx5vyJSSI5AQH0uxhJwIo5Lol9jvQfkaCQUB5KT4ZkSFVQjir84bw/MT2IdpieWr9DT8jLkuGily8z2X4inxGFcQAx+QSYbKv2lgYS6U1ZGk8P/jPQX/fr3KZ+WFVPFNlT7j9EmUNM";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        // Set up color sensors
        dim = hardwareMap.deviceInterfaceModule.get("cdim");
        dim.setDigitalChannelMode(FRONT_LED_CHANNEL, DigitalChannel.Mode.OUTPUT);
        dim.setDigitalChannelMode(REAR_LED_CHANNEL, DigitalChannel.Mode.OUTPUT);
        rgbSensorFront = hardwareMap.colorSensor.get("color_sensor1");
        rgbSensorRear = hardwareMap.colorSensor.get("color_sensor2");
        dim.setDigitalChannelState(FRONT_LED_CHANNEL, ledOn);
        dim.setDigitalChannelState(REAR_LED_CHANNEL, ledOn);

        // Set up motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        lift = hardwareMap.get(DcMotor.class, "lift");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        relicTrackables.activate();
        leftDriveEncCount = 0;
        rightDriveEncCount= 0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Declare local variables
        State state;
        int stateIndex = 0;
        int redFront;
        int redRear;
        int greenFront;
        int greenRear;
        int blueFront;
        int blueRear;
        double leftDist = 0;
        double rightDist = 0;
        double leftDistTotal = 0;
        double rightDistTotal = 0;
        int knockBallDirection = 0;
        double knockBallVector = 0;
        double hueFront;
        double hueRear;
        boolean ballColorKnownFront = FALSE;
        boolean ballColorKnownRear = FALSE;
        boolean isFrontBallRed = FALSE;
        boolean isRearBallRed = FALSE;
        double leftDrivePower = 0;
        double rightDrivePower = 0;
        double leftIntakePower = 0;
        double rightIntakePower = 0;
        double liftPower = 0;

        // List auton actions and move distances
        State[] autonStatesArr = {ARM_DOWN, ID_PICTO, ID_BALL, RESET_ENCODERS, KNOCK_BALL, ARM_UP, MOVE, MOVE,
                                  MOVE, RESET_TIME, DROP_GLYPH, ALL_STOP};

        // Move distances, in inches
        double[] leftDriveMoveDist =  {0, 0, 0, 0,  3, 0,   0,  0,  0, 0, 0, 0};
        double[] rightDriveMoveDist = {0, 0, 0, 0,  3, 0,   0,  0,  0, 0, 0, 0};

        // Drive motor powers
        double[] leftDrivePowerArr =  {0, 0, 0, 0, .7, 0, .85, .5, .7, 0, 0, 0};
        double[] rightDrivePowerArr = {0, 0, 0, 0, .7, 0, .85, .5, .7, 0, 0, 0};

        state = autonStatesArr[stateIndex];

        switch (state) {
            case ID_PICTO:
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//            telemetry.addData("VuMark", "%s visible", vuMark);
                    switch (vuMark) {
                        case LEFT:
                            telemetry.addData("Column", "LEFT");
                            leftDriveMoveDist[6] = 00000;   //Drive Straight
                            rightDriveMoveDist[6] = 00000;  //Drive Straight
                            leftDriveMoveDist[7] = 00000 * PI;   //Turn
                            rightDriveMoveDist[7] = 00000 * PI;   //Turn
                            leftDriveMoveDist[8] = 00000;   //Drive Straight
                            rightDriveMoveDist[8] = 00000;  //Drive Straight
                            break;
                        case CENTER:
                            telemetry.addData("Column", "CENTER");
                            leftDriveMoveDist[6] = 00000;   //Drive Straight
                            rightDriveMoveDist[6] = 00000;  //Drive Straight
                            leftDriveMoveDist[7] = 00000 * PI;   //Turn
                            rightDriveMoveDist[7] = 00000 * PI;   //Turn
                            leftDriveMoveDist[8] = 00000;   //Drive Straight
                            rightDriveMoveDist[8] = 00000;  //Drive Straight
                            break;
                        case RIGHT:
                            telemetry.addData("Column", "RIGHT");
                            leftDriveMoveDist[6] = 00000;   //Drive Straight
                            rightDriveMoveDist[6] = 00000;  //Drive Straight
                            leftDriveMoveDist[7] = 00000 * PI;   //Turn
                            rightDriveMoveDist[7] = 00000 * PI;   //Turn
                            leftDriveMoveDist[8] = 00000;   //Drive Straight
                            rightDriveMoveDist[8] = 00000;  //Drive Straight
                            break;
                        default:
                            telemetry.addData("Column", "Unknown");
                    }
                    stateIndex++;
                } else {
                    telemetry.addData("VuMark", "not visible");
                }
            case ID_BALL:
                if (!ballColorKnownFront) {
                    redFront = rgbSensorFront.red();
                    greenFront = rgbSensorFront.green();
                    blueFront = rgbSensorFront.blue();
                    Color.RGBToHSV((redFront * 255) / 800, (greenFront * 255) / 800, (blueFront * 255) / 800, hsvValuesFront);
                    hueFront = hsvValuesFront[0];

                    if ((redFront > blueFront) && (hueFront < 60)) {
                        telemetry.addData("Front Ball Color", "Red");
                        isFrontBallRed = TRUE;
                        ballColorKnownFront = TRUE;
                    } else if ((redFront < blueFront) && (hueFront > 60)) {
                        telemetry.addData("Front Ball Color", "Blue");
                        isFrontBallRed = FALSE;
                        ballColorKnownFront = TRUE;
                    } else {
                        telemetry.addData("Front Ball Color", "Unknown");
                    }
                }

                if (!ballColorKnownRear) {
                    redRear = rgbSensorRear.red();
                    greenRear = rgbSensorRear.green();
                    blueRear = rgbSensorRear.blue();
                    Color.RGBToHSV((redRear * 255) / 800, (greenRear * 255) / 800, (blueRear * 255) / 800, hsvValuesRear);
                    hueRear = hsvValuesRear[0];

                    if ((redRear > blueRear) && (hueRear < 60)) {
                        telemetry.addData("Rear Ball Color", "Red");
                        isRearBallRed = TRUE;
                        ballColorKnownRear = TRUE;
                    } else if ((redRear < blueRear) && (hueRear > 60)) {
                        telemetry.addData("Rear Ball Color", "Blue");
                        isRearBallRed = FALSE;
                        ballColorKnownRear = TRUE;
                    } else {
                        telemetry.addData("Rear Ball Color", "Unknown");
                    }
                }
                if ((state == ID_BALL) && (ballColorKnownFront == TRUE) && (ballColorKnownRear == TRUE)) {
                    if (isFrontBallRed && !isRearBallRed) {
                        knockBallDirection = 1;
                    } else if (!isFrontBallRed && isRearBallRed) {
                        knockBallDirection = -1;
                    } else {
                        knockBallDirection = 0;
                    }
                    stateIndex++;
                }
                break;
            case RESET_TIME:
                runtime.reset();
                stateIndex++;
                break;
            case RESET_ENCODERS:
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                stateIndex++;
                break;
            case KNOCK_BALL:
                if (knockBallDirection == 0) {
                    stateIndex++;
                } else {
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
            case ARM_DOWN:
                armPosDeg = ARM_DOWN_DEG;
                if (runtime.milliseconds() >= 250) {
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
            default:
                telemetry.addData("Status", "state error: %s", state);
        }

        tiltPos = map(tiltPosDeg, TILT_MIN_POS_DEG, TILT_MAX_POS_DEG, TILT_MIN_POS, TILT_MAX_POS);
        armPos = map(armPosDeg, ARM_MIN_POS_DEG, ARM_MAX_POS_DEG, ARM_MIN_POS, ARM_MAX_POS);

        // Set powers for motors and positions for servos
        leftFrontDrive.setPower(leftDrivePower);
        leftRearDrive.setPower(leftDrivePower);
        rightFrontDrive.setPower(rightDrivePower);
        rightRearDrive.setPower(rightDrivePower);
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
    }

    boolean leftInRange() {
        if ((leftRearDrive.getCurrentPosition() >= leftDriveEncCount - ENC_ERR_RNG) &&
            (leftRearDrive.getCurrentPosition() <= leftDriveEncCount + ENC_ERR_RNG)) {
            return TRUE;
        } else {
            return FALSE;
        }
    }

    boolean rightInRange() {
        if ((rightRearDrive.getCurrentPosition() >= rightDriveEncCount - ENC_ERR_RNG) &&
            (rightRearDrive.getCurrentPosition() <= rightDriveEncCount + ENC_ERR_RNG)) {
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

        leftRearDrive.setTargetPosition(leftDriveEncCount);
        rightRearDrive.setTargetPosition(rightDriveEncCount);

        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}