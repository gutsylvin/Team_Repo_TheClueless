package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

import java.util.Map;

/**
 * Created by Josh on 10/27/2016.
 */

public interface Instruction {

    boolean finished = false;

    void Loop ();
    void Finished ();
    void Init ();
    void FromMap(Map<String, String> map);
}
