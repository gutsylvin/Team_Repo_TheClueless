package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual.Instructions;

import com.google.gson.annotations.Expose;

import org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual.Instructions.Instruction;

/**
 * Created by hsunx on 11/11/2016.
 */

public class LineFollowInstruction extends Instruction {

    final float P_COEFF = 0.1f;
    final float TARGET = 0.6f;

    @Expose
    public double speed;

    @Expose
    public int encoder_count;

    @Override
    public void Init() {
        super.Init();
    }

    @Override
    public void Loop() {
        if (robot.leftMotor.getCurrentPosition() > encoder_count || robot.rightMotor.getCurrentPosition() > encoder_count) {
            Finished();
        }

        double error = (TARGET - robot.opticalDistanceSensor.getLightDetected()) * P_COEFF;
        robot.leftMotor.setPower(speed - error);
        robot.rightMotor.setPower(speed + error);
    }

}
