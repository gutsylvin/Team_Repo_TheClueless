package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual.Instructions;

import com.google.gson.annotations.Expose;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

/**
 * Created by Josh on 10/27/2016.
 */

public abstract class Instruction {

    Robot robot;
    public boolean finished;
    public boolean initialized;

    @Expose
    public String name;

    public abstract void Loop ();
    public void Finished () {
        finished = true;
    }
    public void Init () {
        initialized = true;
        robot = Robot.robot;
    }

    @Override
    public String toString() {
        return super.toString();
    }
}
