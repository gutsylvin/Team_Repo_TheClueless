package org.firstinspires.ftc.teamcode.AutonomousNavigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by hsunx on 10/22/2016.
 */

// TODO replace this with a system which takes a file on the phone and adjusts accordingly

@Autonomous(name = "Match Parameters: Blue", group = "MatchParameters")
public class SetBlueOpMode extends OpMode {
    @Override
    public void start() {
        super.start();
        MatchDetails.color = MatchDetails.TeamColor.BLUE;
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
