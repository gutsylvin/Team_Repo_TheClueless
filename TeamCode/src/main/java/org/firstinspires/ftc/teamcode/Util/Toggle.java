package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by Josh on 12/3/2016.
 */

public abstract class Toggle {

    class GamepadInfo {
        public Gamepad gamepad;
        public Gamepad previous;

        public boolean firstPress;

        public enum Buttons {
            A, B, X, Y,
            D_UP, D_DOWN, D_LEFT, D_RIGHT,
            BUMPER_LEFT, BUMPER_RIGHT
        }
        public boolean getToggled () {

        }
    }

    static GamepadInfo gamepad1;
    static GamepadInfo gamepad2;

    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        Toggle.gamepad1.gamepad = gamepad1;
        Toggle.gamepad2.gamepad = gamepad2;
    }

    // Do all the stuffs here
    public void update () {
        if (gamepad1.firstPress && gamepad1.gamepad.atRest()) {
            gamepad1.firstPress = false;
        }
        if (gamepad2.firstPress && gamepad2.gamepad.atRest()) {
            gamepad2.firstPress = false;
        }
        if (gamepad1.firstPress || gamepad2.firstPress) {
            return;
            gamepad1.gamepad.
        }
    }

    public static void updatesFinished() {
        try {
            if (gamepad1.previous == null && !gamepad1.gamepad.atRest()){
                gamepad1.firstPress = true;
            }
            if (gamepad2.previous == null && !gamepad2.gamepad.atRest()) {
                gamepad2.firstPress = true;
            }
            if (gamepad1.gamepad != null) {
                gamepad1.previous.copy(gamepad1.gamepad);
            }
            if (gamepad2.gamepad != null) {
                gamepad2.previous.copy(gamepad2.gamepad);
            }
        }
        catch (RobotCoreException e) {
            RobotLog.e("Error with gamepad toggles");
        }
    }
}
