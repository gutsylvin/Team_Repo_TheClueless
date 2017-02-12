package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;
import org.lasarobotics.vision.util.color.Color;


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
    public ColorSensor adafruitColor;

    @Override
    public void init() {
        if (Robot.robot == null) {
            robot = new Robot();
            robot.init(hardwareMap, telemetry);
        }
        else {
            robot = Robot.robot;
        }
        opticalDistanceSensor = robot.opticalDistanceSensor;
        gyro = robot.gyro;
        color = robot.colorSensor;
        adafruitColor = robot.adafruitI2cColorSensor;
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
        telemetry.addData("Color sensor greyscale", getGreyscale(color));
        telemetry.addData("Color sensor grey scaled", getGreyscale(color) / 255);

        telemetry.addData("Adafruit Color sensor red", adafruitColor.red());
        telemetry.addData("Adafruit Color sensor green", adafruitColor.green());
        telemetry.addData("Adafruit Color sensor blue", adafruitColor.blue());
        telemetry.addData("Adafruit Color sensor grey", adafruitColor.alpha());
        telemetry.addData("Adafruit Color sensor greyscale", getGreyscale(adafruitColor));
        telemetry.addData("Adafruit Color sensor grey scaled", getGreyscale(adafruitColor) / 255);
        for (DcMotor motor : motors) {
            telemetry.addData("Encoder count", motor.getCurrentPosition());
        }
        telemetry.update();
    }

    public double getGreyscale(ColorSensor c) {
        return 0.299 * c.red() + 0.587 * c.green() + 0.114 * c.blue();
    }
}
