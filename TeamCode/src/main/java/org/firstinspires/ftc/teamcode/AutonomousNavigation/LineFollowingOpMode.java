package org.firstinspires.ftc.teamcode.AutonomousNavigation;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;


/**
 * Created by hsunx on 11/26/2016.
 */

@Autonomous (name = "Line Follow Test", group = "testing")
public class LineFollowingOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap, telemetry);

        robot.opticalDistanceSensor.enableLed(true);

        while (!isStarted()) {
            telemetry.addData("ods reading", robot.opticalDistanceSensor.getLightDetected());
            telemetry.addData("ods raw reading", robot.opticalDistanceSensor.getRawLightDetected());
            telemetry.update();
            idle();
        }
    }
}
