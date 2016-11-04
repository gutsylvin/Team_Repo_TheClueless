package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

/**
 * Created by Josh on 10/27/2016.
 */

public abstract class Instruction {

    Robot robot;
    public boolean finished;
    public boolean initialized;

    public String name;

    public abstract void Loop ();
    public abstract void Finished ();
    public void Init () {
        initialized = true;
        robot = Robot.robot;
    }
}
