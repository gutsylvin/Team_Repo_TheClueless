package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

/**
 * Created by hsunx on 10/21/2016.
 */

public class MovementInstruction {
    public float angle;
    public float distance;
    public float power;

    public MovementInstruction (float angle, float distance, float power) {
        this.angle = angle;
        this.distance = distance;
        this.power = power;
    }
}
