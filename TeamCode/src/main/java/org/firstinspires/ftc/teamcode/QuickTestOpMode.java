package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

import static org.firstinspires.ftc.teamcode.RobotHardware.Robot.robot;

/**
 * Created by hsunx on 10/29/2016.
 */
@Autonomous (name = "Tester", group = "Test")
public class QuickTestOpMode extends LinearOpMode {

    final double P_DRIVE_COEFF = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap, telemetry);

        waitForStart();

        gyroDriveSimple(0.25, 19305, 30, 5);
    }

    public void gyroDriveSimple (double speed, double distance, double angle, double timeout) throws InterruptedException{

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double leftSpeed;
        double rightSpeed;

        double startTime = time;

        while (opModeIsActive() && time - startTime < timeout) {
            double error = angle - robot.gyro.getIntegratedZValue();
            double offset = error * P_DRIVE_COEFF;
            robot.leftMotor.setPower(speed + offset);
            robot.rightMotor.setPower(speed - offset);
            telemetry.addData("angle", robot.gyro.getIntegratedZValue());
            telemetry.addData("error", error);
            telemetry.update();
            idle();
        }
    }
}
