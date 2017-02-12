package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual.Instructions;

import com.google.gson.annotations.Expose;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.AutonomousNavigation.MatchDetails;

/**
 * Created by hsunx on 11/11/2016.
 */

public class PushBeaconButtonInstruction extends Instruction {

    // Assuming use of HS-485HB servo
    final double ENDPOINT = 230/255;

    @Expose
    public double time;

    @Expose
    public double speed;

    @Override
    public void Init() {
        super.Init();

        // TODO there isn't any fallback if they are equal! plus this might be kinda inaccurate, oh well.
        if (robot.colorSensor.blue() > robot.colorSensor.red()) {
            if (MatchDetails.color == MatchDetails.TeamColor.BLUE) {
                // Gud.
                robot.rightPushServo.setPosition(ENDPOINT/2);
            }
            else {
                robot.leftPushServo.setPosition(ENDPOINT/2);
            }
        }
        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                // Gud.
                robot.rightPushServo.setPosition(ENDPOINT/2);
            }
            else {
                robot.leftPushServo.setPosition(ENDPOINT/2);
            }
        }

        try {
            WaitForServos();
        }
        catch (InterruptedException e) {
            RobotLog.e("wtf why was the thread interrupted while waiting for servos");
        }

        //TODO make a time based motor instruction
        robot.leftMotor.setPower(0.75);
        robot.rightMotor.setPower(0.75);
        robot.timer.reset();
    }

    void WaitForServos () throws InterruptedException{
        wait(800);
    }

    @Override
    public void Loop() {
        if (robot.timer.milliseconds() > time) {
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.leftPushServo.setPosition(0);
            robot.rightPushServo.setPosition(230/255);
            Finished();
        }
    }
}
