package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.AutonomousNavigation.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

import java.lang.reflect.Field;
import java.util.ArrayList;


/**
 * Created by hsunx on 1/20/2017.
 */
@Autonomous(name = "Sensors", group = "Sensors")
public class SensorsOpMode extends OpMode {
    public Robot robot;

    public DcMotor[] motors;
    public OpticalDistanceSensor opticalDistanceSensor;
    public GyroSensor gyro;
    public ColorSensor color;
    public AdafruitI2cColorSensor adafruitI2cColorSensor;

    @Override
    public void init() {
        if (Robot.robot == null) {
            robot = new Robot();
            robot.init(hardwareMap, telemetry);
        }
        else {
            robot = Robot.robot;
        }
        //adafruitI2cColorSensor = robot.adafruitColorSensor;
        opticalDistanceSensor = robot.opticalDistanceSensor;
        gyro = robot.gyro;
        color = robot.colorSensor;
        motors = new DcMotor[] {robot.leftMotor, robot.rightMotor, robot.conveyorMotor, robot.scissorliftMotor, robot.leftShootMotor, robot.rightShootMotor, robot.scissorLiftArmMotor, robot.ballCollectionMotor};

    }

    @Override
    public void loop() {
        telemetry.addData("ODS Reading", opticalDistanceSensor.getLightDetected());
        telemetry.addData("Gyro reading", gyro.getHeading());
        telemetry.addData("Color sensor red", color.red());
        telemetry.addData("Color sensor green", color.green());
        telemetry.addData("Color sensor blue", color.blue());
        telemetry.addData("Color sensor grey", color.alpha());

        //telemetry.addData("color sensor adafruit", "red: " + adafruitI2cColorSensor.red()
        //+ " blue: " + adafruitI2cColorSensor.blue() + " green: " + adafruitI2cColorSensor.green());
        for (DcMotor motor : motors) {
            telemetry.addData("Encoder count", motor.getCurrentPosition());
        }
        telemetry.update();
    }
}
