package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by hsunx on 10/21/2016.
 */

public class BasicInterpreter extends InstructionInterpreter {
    // Requires gyro
    GyroSensor gyro;

    // Above this value, the robot will stop and turn
    float angleThreshhold = 2.5f;

    public BasicInterpreter (DcMotor left, DcMotor right, GyroSensor sensor) {
        super (left, right);
        gyro = sensor;
    }

    @Override
    public boolean Finished() {
        return false;
    }

    @Override
    public void ExecuteInstruction() {

    }
}
