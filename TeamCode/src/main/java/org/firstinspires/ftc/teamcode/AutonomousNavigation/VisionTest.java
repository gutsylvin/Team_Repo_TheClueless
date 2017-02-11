package org.firstinspires.ftc.teamcode.AutonomousNavigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by hsunx on 2/10/2017.
 */
@Autonomous(name = "Vision Test", group = "vision")
public class VisionTest extends LinearVisionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //region ftcvision
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();

        this.setCamera(Cameras.PRIMARY);

        this.setFrameSize(new Size(900, 900));

        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        //endregion

        while (opModeIsActive()) {
            tryAnalysis();
        }
    }
    public void tryAnalysis() {
        Beacon.BeaconAnalysis analysis = beacon.getAnalysis();
        if (analysis.isLeftKnown()) {
            if(analysis.isLeftRed()) {
                telemetry.addData("Left", "Red");
            }
            else if (analysis.isLeftBlue()) {
                telemetry.addData("Left", "Blue");
            }
            else {
                telemetry.addData("Left", "Unknown");
            }
        }
        else {
            telemetry.addData("Left", "Unknown");
        }
        if (analysis.isRightKnown()) {
            if(analysis.isRightRed()) {
                telemetry.addData("Right", "Red");
            }
            else if (analysis.isRightBlue()) {
                telemetry.addData("Right", "Blue");
            }
            else {
                telemetry.addData("Right", "Unknown");
            }
        }
        else {
            telemetry.addData("Right", "Unknown");
        }
        telemetry.update();
    }

}
