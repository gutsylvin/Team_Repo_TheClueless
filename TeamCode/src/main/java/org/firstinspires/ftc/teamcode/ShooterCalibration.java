package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.AutonomousNavigation.MatchDetails;
import org.firstinspires.ftc.teamcode.RobotHardware.Robot;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.RobotHardware.Robot.robot;

/**
 * Created by hsunx on 10/29/2016.
 */
@Autonomous (name = "Encoder Shooting Calibration", group = "Test")
public class ShooterCalibration extends LinearOpMode {

    final double timeoutMs = 15.5 * 1000;

    final int DELAY_SETTLE = 700;
    final int DELAY_MEASURE = 1000;

    final double TOLERANCE = 20;

    final double LOWER_BOUND = 0.35;
    final double UPPER_BOUND = 0.88;

    double currentLowerBound = LOWER_BOUND;
    double currentUpperBound = UPPER_BOUND;

    public static double SHOOTING_SPEED = 0;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap, telemetry);
        timer = new ElapsedTime();
        waitForStart();
        robot.conveyorMotor.setPower(1);
        //robot.ballCollectionMotor.setPower(1);
        if (MatchDetails.color == MatchDetails.TeamColor.RED) {
            setEncoderSpeed(2360, robot.leftShootMotor, robot.rightShootMotor);
        }
        else if (MatchDetails.color == MatchDetails.TeamColor.BLUE) {
            setEncoderSpeed(2360, robot.leftShootMotor, robot.rightShootMotor);
        }
        robot.conveyorMotor.setPower(0);
        //robot.ballCollectionMotor.setPower(0);
        while (opModeIsActive());
    }

    boolean within (double value, double target, double tolerance) {
        return (value < (target + tolerance)) && (value > (target - tolerance));
    }

    public void setEncoderSpeed(int encoderSpeed, DcMotor motorLeft, DcMotor motorRight) throws InterruptedException{
        timer.reset();

        double previousEncoderValue;
        double previousTimeValue;
        double observedEncoderSpeed = 0;
        motorLeft.setPower((currentLowerBound + currentUpperBound) / 2);
        motorRight.setPower((currentLowerBound + currentUpperBound) / 2);
        while (opModeIsActive() && timer.milliseconds() < timeoutMs && (!within(observedEncoderSpeed, encoderSpeed, TOLERANCE))) {

            motorLeft.setPower((currentLowerBound + currentUpperBound) / 2);
            motorRight.setPower((currentLowerBound + currentUpperBound) / 2);
            Thread.sleep(DELAY_SETTLE);
            previousTimeValue = timer.time();
            previousEncoderValue = motorLeft.getCurrentPosition();
            Thread.sleep(DELAY_MEASURE);

            observedEncoderSpeed = (motorLeft.getCurrentPosition() - previousEncoderValue) / (timer.time() - previousTimeValue);
            if (observedEncoderSpeed > encoderSpeed) {
                currentUpperBound = (currentLowerBound + currentUpperBound) / 2;
            }
            else if (observedEncoderSpeed < encoderSpeed){
                currentLowerBound = (currentLowerBound + currentUpperBound) / 2;
            }
            telemetry.addData("speed", observedEncoderSpeed);
            telemetry.addData("current power", (currentLowerBound + currentUpperBound) / 2);
            telemetry.update();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

        telemetry.addData("speed", observedEncoderSpeed);
        SHOOTING_SPEED = (currentLowerBound + currentUpperBound) / 2;
    }

}
