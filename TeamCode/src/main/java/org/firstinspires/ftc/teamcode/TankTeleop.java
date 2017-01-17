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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;
import org.firstinspires.ftc.teamcode.Util.MotorToggle;
import org.firstinspires.ftc.teamcode.Util.ServoToggle;
import org.firstinspires.ftc.teamcode.Util.Toggle;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Pushbot: Teleop Tank", group = "TeleOp")
public class TankTeleop extends OpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot(); // use the class created to define a Robot's hardware class

    // Assuming HS-485HB servo
    final float ENDPOINT = 230 / 255;
    final float PUSH = ENDPOINT / 2;
    double shootSpeed;
    final float SCISSORLIFT_SERVO_DOWN = 200 / 255;

    private boolean leftBeacon = false;
    private boolean rightBeacon = false;

    private boolean scissorliftServos = false;

    private boolean sweeper = false;

    private boolean shooting = false;

    private boolean conveyor = false;

    private boolean conveyorGate = false;

    private boolean armReleasers = false;

    private boolean reverse = false;

    private Gamepad previousGamepad1;
    private Gamepad previousGamepad2;

    private double timeSinceLastTap;

    private double sweeperSpeed = 0.75;

    private boolean firstLoop = true;

    private double lastTime;

    private double scissorliftArmMaxSpeed = 0.5;

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
        robot.leftPushServo.setPosition(0);
        robot.rightPushServo.setPosition(0.961); // 245/255
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.leftPushServo.setPosition(0);
        robot.rightPushServo.setPosition(0.961); // 245/255
        recalculateSpeed();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double deltaTime = time - lastTime;
        lastTime = time;

        if (firstLoop) {
            firstLoop = false;
            robot.leftPushServo.setPosition(0);
            robot.rightPushServo.setPosition(0.961); // 245/255
            robot.conveyorGate.setPosition(0.863);
            try {
                previousGamepad1.copy(gamepad1);
                previousGamepad2.copy(gamepad2);
            } catch (RobotCoreException e) {
                telemetry.addData("ERROR", e.toString());
            }
            return;
        }

        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = (-1) * (reverse ? gamepad1.right_stick_y : gamepad1.left_stick_y);
        right = (-1) * (reverse ? gamepad1.left_stick_y : gamepad1.right_stick_y);

        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);

        Toggle[] toggles = {new ServoToggle(Toggle.Button.BUMPER_RIGHT, robot.armReleasers, new double[]{0, 1}, Toggle.GamepadId.User1),
                            new ServoToggle(Toggle.Button.X, robot.leftPushServo, new double[]{0, 0.431}, Toggle.GamepadId.User1),
                            new ServoToggle(Toggle.Button.B, robot.rightPushServo, new double[]{0.961, 0.5}, Toggle.GamepadId.User1),
                            new ServoToggle(Toggle.Button.BUMPER_RIGHT, robot.conveyorGate, new double[] {0, 0.863}, Toggle.GamepadId.User2),
                            new MotorToggle(Toggle.Button.A, robot.ballCollectionMotor, new double[] {-1, 0}, Toggle.GamepadId.User2),
                            new MotorToggle(Toggle.Button.BUMPER_LEFT, new DcMotor[]{robot.leftShootMotor, robot.rightShootMotor}, new double[]{0.75, 0}, Toggle.GamepadId.User2),
                            new MotorToggle(Toggle.Button.X, robot.conveyorMotor, new double[]{1, 0}, Toggle.GamepadId.User2)};

        if (gamepad1.right_bumper) {
            if (!previousGamepad1.right_bumper) {
                armReleasers = !armReleasers;
            }
        }
        // Beacon pushers
        if (gamepad1.x) {
            if (!previousGamepad1.x) {
                leftBeacon = !leftBeacon;
            }

        }
        if (gamepad1.b) {
            if (!previousGamepad1.b) {
                rightBeacon = !rightBeacon;
            }
        }

        if (gamepad1.left_bumper) {
            if (!previousGamepad1.left_bumper) {
                ReverseMotors();
                reverse = !reverse;
            }
        }

        timeSinceLastTap += deltaTime;

        // Double tap to activate scissor lift servos
        if (gamepad2.b) {
            if (!previousGamepad2.b) {
                if (timeSinceLastTap < 1.5) {
                    scissorliftServos = !scissorliftServos;
                }
                else {
                    timeSinceLastTap = 0;
                }
            }
        }

        // Sweeper toggle
        if (gamepad2.a) {
            if (!previousGamepad2.a) {
                sweeper = !sweeper;
            }
        }

        // Shooter toggle
        if (gamepad2.left_bumper) {
            if (!previousGamepad2.left_bumper) {
                shooting = !shooting;
            }
        }

        if (gamepad2.x) {
            if (!previousGamepad2.x) {
                conveyor = !conveyor;
            }
        }

        if (gamepad2.right_bumper) {
            if (!previousGamepad2.right_bumper) {
                conveyorGate = !conveyorGate;
            }
        }

        if (gamepad2.y) {
            if (!previousGamepad2.y) {
                shooting = false;
                conveyor = false;
                conveyorGate = true;
            }
        }

        if (!shooting && !conveyor && !sweeper && (robot.leftMotor.getPower() == 0) && (robot.rightMotor.getPower() == 0)) {
            recalculateSpeed();
        }

        float scissorLift = -gamepad2.left_stick_y;
        if (scissorLift != 0) {
            conveyorGate = true;
        }

        robot.scissorliftMotor.setPower(scissorLift);

        robot.leftPushServo.setPosition(leftBeacon ? 0.431 /*110/255*/ : 0);
        robot.rightPushServo.setPosition(rightBeacon ? 0.5 : 0.961/*245/255*/);

        double scissorLiftArmSpeed = (gamepad2.left_trigger - gamepad2.right_trigger) * (scissorliftArmMaxSpeed);
        robot.scissorLiftArmMotor.setPower(scissorLiftArmSpeed);

        robot.ballCollectionMotor.setPower(sweeper ? sweeperSpeed : 0);

        robot.conveyorGate.setPosition(conveyorGate ? 0 : 0.625);

        robot.armReleasers.setPosition(armReleasers ? 1 : 0);

        // TODO does this even do anything
        if (robot.leftShootMotor == null || robot.rightShootMotor == null) {
            telemetry.addData("fuck", "fuck");
        }

        shootBalls(shooting);

        recalculateSpeed();

        robot.conveyorMotor.setPower(conveyor ? 1 : 0);
        try {
            previousGamepad1.copy(gamepad1);
            previousGamepad2.copy(gamepad2);
        } catch (Exception exception) {
            telemetry.addData("gamepad error", exception.getMessage());
        }

        telemetry.addData("left joystick", left);
        telemetry.addData("right joystick", right);
        telemetry.addData("left motor", robot.leftMotor.getPower());
        telemetry.addData("right motor", robot.rightMotor.getPower());
        telemetry.addData("beacon", "left: " + leftBeacon + " right: " + rightBeacon);
        telemetry.addData("left push", robot.leftPushServo.getPosition());
        telemetry.addData("right push", robot.rightPushServo.getPosition());
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }

    void recalculateSpeed() {
        double voltage = robot.voltageSensor.getVoltage();

        if (voltage >= 13.5) {
            shootSpeed = 0.465;
        } else if (voltage <= 12) {
            shootSpeed = 0.55;
        } else if (voltage <= 11.5) {
            shootSpeed = 0.6;
        } else {
            shootSpeed = -0.15829 * (Math.pow(voltage, 3)) + 5.9856 * (Math.pow(voltage, 2)) + -75.445 * voltage + 317.47;
        }
    }

    void shootBalls(boolean shooting) {
        if (shooting) {
            robot.shoot(true, shootSpeed);
        }
        else {
            robot.shoot(false);
        }

    }

    private void ReverseMotors() {
        robot.leftMotor.setDirection(ReverseDirection(robot.leftMotor.getDirection()));
        robot.rightMotor.setDirection(ReverseDirection(robot.rightMotor.getDirection()));
    }

    private DcMotorSimple.Direction ReverseDirection(DcMotorSimple.Direction direction) {
        return direction == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }
}
