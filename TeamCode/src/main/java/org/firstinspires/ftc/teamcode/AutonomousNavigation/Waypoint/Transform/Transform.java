package org.firstinspires.ftc.teamcode.AutonomousNavigation.Waypoint.Transform;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by hsunx on 10/15/2016.
 */

public class Transform {

    // Follows the FTC Field Coordinate system

    // Position
    public float x, y, z;

    // Rotation (euler angles)
    public float eX, eY, eZ;

    public Transform (float x, float y, float z, float eX, float eY, float eZ) {

        this.x = x;
        this.y = y;
        this.z = z;
        this.eX = x;
        this.eY = y;
        this.eZ = z;

    }

    public static double Distance (Transform first, Transform second) {
        return (Math.sqrt( Math.pow( ( second.x - first.x),2)) + Math.pow( ( second.y - first.y ), 2) + Math.pow( ( second.z - first.z), 2));
    }

    // Only takes into account the x and y values
    public static double TopDownDistance (Transform first, Transform second) {
        return Math.sqrt (Math.pow( (second.x - first.x), 2) + Math.pow( (second.y - first.y), 2));
    }

    public static Transform FromString (String transform) {
        return FromString(transform, ",");
    }
    public static double TopDownAngleBetweenPoints (Transform from, Transform to) {
        return Math.atan2(to.x - from.x, to.y - from.y);
    }
    public static Transform FromString (String transform, String seperator) {

        String[] data = transform.split(seperator);
        float[] floatData = new float[data.length];
        for (int i = 0; i < data.length; i++) {
            floatData[i] = Float.parseFloat(data[i]);
        }
        return new Transform (floatData[0], floatData[1], floatData[2], floatData[3], floatData[4], floatData[5]);
    }

    public static Transform FromVuforiaOpenGLMatrix (OpenGLMatrix matrix) {
        String transform = matrix.formatAsTransform();
        transform.replaceAll("EXTRINSIC", "");
        transform.replaceAll("{", "");
        transform.replaceAll("}", "");
        transform.replaceAll("XYZ", "");
        return FromString(transform, " ");
    }

}
