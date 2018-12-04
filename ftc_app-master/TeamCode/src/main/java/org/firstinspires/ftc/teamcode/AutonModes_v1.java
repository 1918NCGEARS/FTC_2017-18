package org.firstinspires.ftc.teamcode;

/**
 * Created by jtuuk on 11/7/2017.
 */

public class AutonModes_v1 {

    public AutonModes_v1(){

    }

    public enum State {
        ID_BALL, KNOCK_BALL, ARM_UP, ARM_DOWN, MOVE, RESET_TIME, RESET_ENCODERS, DROP_GLYPH,
        ALL_STOP, TIME_DELAY, LIFT_UP, LIFT_DOWN, TILT_UP, TILT_DOWN
    }

}
