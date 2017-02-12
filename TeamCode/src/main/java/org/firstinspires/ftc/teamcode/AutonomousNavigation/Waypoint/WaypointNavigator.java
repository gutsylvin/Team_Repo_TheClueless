package org.firstinspires.ftc.teamcode.AutonomousNavigation.Waypoint;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.AutonomousNavigation.Waypoint.Transform.Transform;

/**
 * Created by hsunx on 10/14/2016.
 */

public abstract class WaypointNavigator {
    protected Transform currentPosition;
    protected Transform lastKnownPosition;

    // Radius around the destination where the navigator can report finished
    protected float marginOfError;

    protected VuforiaPositionFinder finder;

    protected WaypointController controller;

    protected Waypoint target;

    public WaypointNavigator (Waypoint initialTarget) {
        target = initialTarget;
    }

    public void SetNewTarget (Waypoint newTarget) {
        target = newTarget;
    }


    // Leave the implementation of this to the children classes
    public abstract void DriveToTarget ();

    public boolean Finished () {
        if (Transform.Distance(currentPosition, target.point) < marginOfError) {
            return true;
        }
        return false;
    }



}
