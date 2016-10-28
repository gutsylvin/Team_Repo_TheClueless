package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import java.util.Map;

/**
 * Created by hsunx on 10/21/2016.
 */

public class MovementInstruction implements Instruction {
    public float angle;
    public float distance;
    public float power;

    @Override
    public void FromMap(Map<String, String> map) {

    }

    @Override
    public void Init() {

    }

    @Override
    public void Loop() {

    }

    @Override
    public boolean Finished() {
        return false;
    }

}
