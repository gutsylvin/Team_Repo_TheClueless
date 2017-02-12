package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by hsunx on 12/4/2016.
 */

public class MotorToggle extends Toggle {

    DcMotor[] motors;
    double[] speeds;
    int index;

    public MotorToggle (Button button, DcMotor motor, double[] speeds, GamepadId gamepad) {
        super(button, gamepad);
        new MotorToggle (button, new DcMotor[] {motor}, speeds, gamepad);
    }

    public MotorToggle (Button button, DcMotor[] motors, double[] speeds, GamepadId gamepad) {
        super(button, gamepad);

        // Make sure the speeds are valid
        for (int i = 0; i < speeds.length; i++) {
            speeds[i] = Range.clip(speeds[i], -1, 1);
        }

        this.speeds = speeds;
        this.motors = motors;
    }

    @Override
    void toggled() {
        for (DcMotor motor : motors) {
            motor.setPower(speeds[index]);
        }
        index++;
    }
}
