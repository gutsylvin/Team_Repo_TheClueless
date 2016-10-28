package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutonomousNavigation.AutonomousOpMode;

/**
 * Created by hsunx on 10/21/2016.
 */

public class BasicInterpreter extends InstructionInterpreter {
    // Requires gyro
    ModernRoboticsI2cGyro gyro;
    AutonomousOpMode opmode;

    public BasicInterpreter (DcMotor left, DcMotor right, GyroSensor sensor, AutonomousOpMode opmode){
        super (left, right);
        this.opmode = opmode;
    }

    @Override
    public void ExecuteInstruction() {
        // opmode.gyroDrive();
    }

    @Override
    public void Finished() {

    }
}
