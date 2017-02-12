package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.AutonomousNavigation.MatchDetails;
import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

/**
 * Created by hsunx on 10/22/2016.
 */
@Autonomous(name = "Match Parameters: Red", group = "MatchParameters")
public class SetRedOpMode extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{

        MatchDetails.color = MatchDetails.TeamColor.RED;
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
