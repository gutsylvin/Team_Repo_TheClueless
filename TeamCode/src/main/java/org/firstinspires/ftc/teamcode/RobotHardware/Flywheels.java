package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.MotorToggle;

/**
 * Created by hsunx on 12/30/2016.
 */

public class Flywheels {

    double speed = 0.5;

    static final int ENCODER_MAX = 2800; // Neverest motors -- no gearing

    DcMotor[] flywheels;

    public boolean started;

    Thread flywheelsMonitor;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void start() {
        if (flywheels == null) {
            return;
        }
        for (DcMotor motor : flywheels) {
            motor.setPower(speed);
        }

        started = true;
    }

    public void setFlywheels(DcMotor... motors) {
        flywheels = motors;
        for (DcMotor motor : flywheels) {
            motor.setMaxSpeed(ENCODER_MAX);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void stop() {
        if (flywheels == null) {
            return;
        }
        for (DcMotor motor : flywheels) {
            motor.setPower(0);
        }

        started = false;
    }


}
