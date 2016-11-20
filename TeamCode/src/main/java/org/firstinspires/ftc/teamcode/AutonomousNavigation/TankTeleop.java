/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.AutonomousNavigation;

import com.qualcomm.hardware.logitech.LogitechGamepadF310;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="TeleOp")
public class TankTeleop extends OpMode{

    /* Declare OpMode members. */
    Robot robot       = new Robot(); // use the class created to define a Robot's hardware class

    // Assuming HS-485HB servo
    final float ENDPOINT = 230/255;
    final float PUSH = ENDPOINT/2;

    final float SCISSORLIFT_SERVO_DOWN = 200/255;

    boolean leftBeacon = false;
    boolean rightBeacon = false;

    boolean sweeper = false;

    boolean shooting = false;

    boolean conveyor = false;

    Gamepad previousGamepad1;
    Gamepad previousGamepad2;

    double timeOfLastTap;

    double sweeperSpeed = 1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.rightScissorliftServo.setPosition(0);
        robot.leftScissorliftServo.setPosition(0);
        robot.leftPushServo.setPosition(0);
        robot.rightPushServo.setPosition(0);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);

        // Beacon pushers
        if (gamepad1.a) {
            if (previousGamepad1 != null) {
                if (!previousGamepad1.a) {
                    leftBeacon = !leftBeacon;
                }
            }
        }
        if (gamepad1.b) {
            if (previousGamepad1 != null) {
                if (!previousGamepad1.b) {
                    rightBeacon = !rightBeacon;
                }
            }
        }

        // Double tap to activate scissor lift servos
        if (gamepad1.x && previousGamepad1 != null) {
            if (previousGamepad1.x) {
                if ((robot.timer.milliseconds() - timeOfLastTap) < 1.25 * 1000) {
                    robot.leftScissorliftServo.setPosition(0.825);
                    robot.rightScissorliftServo.setPosition(0.95);
                }
                timeOfLastTap = robot.timer.milliseconds();
            }
        }

        // TODO "Arm Releasers"

        // Sweeper toggle
        if (gamepad2.a) {
            if (previousGamepad2 != null)
            {
                if (!previousGamepad2.a) {
                    sweeper = !sweeper;
                }
            }
        }

        // Shooter toggle
        if (gamepad2.b) {
            if (previousGamepad2 != null)
            {
                if (!previousGamepad2.b) {
                    shooting = !shooting;
                }
            }
        }

        if (gamepad2.x) {
            if (previousGamepad2 != null)
            {
                if (!previousGamepad2.x) {
                    conveyor = !conveyor;
                }
            }
        }

        float scissorLift = -gamepad2.left_stick_y;

        robot.scissorliftMotor.setPower(scissorLift);

        robot.leftPushServo.setPosition(leftBeacon ? 0.425 : 0);
        robot.rightPushServo.setPosition(rightBeacon ? 0.5 : 0);

        robot.ballCollectionMotor.setPower(sweeper ? sweeperSpeed : 0);

        if (robot.leftShootMotor == null || robot.rightShootMotor == null) {
            telemetry.addData("fuck", "fuck");
        }

        robot.leftShootMotor.setPower(shooting ? 1 : 0);
        robot.rightShootMotor.setPower(shooting ? 1 : 0);

        // robot.conveyorMotor.setPower(conveyor ? 1 : 0);
        try {
            previousGamepad1.copy(gamepad1);
            previousGamepad2.copy(gamepad2);
        }
        catch (Exception exception) {
            telemetry.addData("gamepad error", exception.getMessage());
        }

        telemetry.addData("left joystick", left);
        telemetry.addData("right joystick", right);
        telemetry.addData("left motor", robot.leftMotor.getPower());
        telemetry.addData("right motor", robot.rightMotor.getPower());
        telemetry.addData("beacon", "left: " + leftBeacon + " right: " + rightBeacon);
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
