package org.firstinspires.ftc.teamcode.AutonomousNavigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by hsunx on 10/22/2016.
 */
@Autonomous(name = "Match Parameters: Red", group = "MatchParameters")
public class SetRedOpMode extends OpMode {
    @Override
    public void start() {
        super.start();
        MatchDetails.color = MatchDetails.TeamColor.RED;
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
