package org.firstinspires.ftc.teamcode.AutonomousNavigation.Waypoint;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by hsunx on 10/14/2016.
 */

// Adds stack of waypoints for robot navigation
// Relies on an abstract "WaypointNavigator" to do the actual driving
public class WaypointController {

    Waypoint[] waypoints;
    int waypointIndex;

    Boolean started;
    Boolean finished;

    WaypointNavigator navigator;

    // Instantiates class from an array of waypoints
    public WaypointController (Waypoint[] waypoints, WaypointNavigator navigator) {
        this.waypoints = waypoints;
        this.navigator = navigator;
    }

    // TODO Support loading from a WaypointLoader class (which would also specify the WaypointNavigator to use)

    public void Start () {
        started = true;
    }

    public void Finish () {
        finished = true;
    }

    // Call this in the loop -- loop de loop!
    public void Update () {
        if (finished) {
            return;
        }

        if (navigator.Finished()) {
            if (waypointIndex == waypoints.length - 1) {
                Finish();
            }
            else {
                WaypointReached();
            }
        }
        navigator.DriveToTarget();
    }

    public void WaypointReached () {
        waypointIndex++;
        navigator.SetNewTarget(waypoints[waypointIndex]);
    }


}
