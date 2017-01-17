package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.RobotHardware.Robot.robot;

/**
 * Created by hsunx on 10/29/2016.
 */
@Autonomous (name = "Tester", group = "Test")
public class QuickTestOpMode extends LinearOpMode {

    final double P_DRIVE_COEFF = 0.05;

    double timeoutMs = 12.5 * 1000;

    final int DELAY = 1600;

    final double TOLERANCE = 20;

    final double LOWER_BOUND = 0.25;
    final double UPPER_BOUND = 0.75;

    double currentLowerBound = LOWER_BOUND;
    double currentUpperBound = UPPER_BOUND;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap, telemetry);
        timer = new ElapsedTime();
        waitForStart();
        robot.conveyorMotor.setPower(1);
        setEncoderSpeed(2050, robot.leftShootMotor, robot.rightShootMotor);
        robot.conveyorMotor.setPower(0);
        while (opModeIsActive());
    }

    boolean within (double value, double target, double tolerance) {
        return (value < (target + tolerance)) && (value > (target - tolerance));
    }

    public void setEncoderSpeed(int encoderSpeed, DcMotor motor, DcMotor other) throws InterruptedException{
        timer.reset();

        double previousEncoderValue = motor.getCurrentPosition();
        double previousTimeValue = timer.time();
        double observedEncoderSpeed = 0;
        motor.setPower((currentLowerBound + currentUpperBound) / 2);
        other.setPower((currentLowerBound + currentUpperBound) / 2);
        while (opModeIsActive() && timer.milliseconds() < timeoutMs && (!within(observedEncoderSpeed, encoderSpeed, TOLERANCE))) {
            if ((motor.getCurrentPosition() - previousEncoderValue) != 0) {
                // We have gotten an update!
                observedEncoderSpeed = (motor.getCurrentPosition() - previousEncoderValue) / (timer.time() - previousTimeValue);
                if (observedEncoderSpeed > encoderSpeed) {
                    currentUpperBound = (currentLowerBound + currentUpperBound) / 2;
                    motor.setPower((currentLowerBound + currentUpperBound) / 2);
                    other.setPower((currentLowerBound + currentUpperBound) / 2);
                }
                else if (observedEncoderSpeed < encoderSpeed){
                    currentLowerBound = (currentLowerBound + currentUpperBound) / 2;
                    motor.setPower((currentLowerBound + currentUpperBound) / 2);
                    other.setPower((currentLowerBound + currentUpperBound) / 2);
                }

                previousEncoderValue = motor.getCurrentPosition();
                previousTimeValue = timer.time();

                Thread.sleep(DELAY);

            }
            telemetry.addData("speed", observedEncoderSpeed);
            telemetry.addData("current power", (currentLowerBound + currentUpperBound) / 2);
            telemetry.update();
        }

        motor.setPower(0);
        other.setPower(0);

        telemetry.addData("speed", observedEncoderSpeed);
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
