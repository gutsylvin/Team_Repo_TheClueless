package org.firstinspires.ftc.teamcode.AutonomousNavigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

/**
 * Created by hsunx on 10/22/2016.
 */

// TODO replace this with a system which takes a file on the phone and adjusts accordingly

@Autonomous(name = "Match Parameters: Blue", group = "MatchParameters")
public class SetBlueOpMode extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{

        MatchDetails.color = MatchDetails.TeamColor.BLUE;
        robot = new Robot();
        robot.init(hardwareMap, telemetry);
        GyroSensor gyro = robot.gyro;
        while (gyro.getHeading() != 0) {
            gyro.calibrate();

            while (gyro.isCalibrating()) {
                Thread.sleep(50);
                idle();
            }
        }
    }

}
