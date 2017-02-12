package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by Josh on 12/3/2016.
 */


public abstract class Toggle {

    // TODO Maybe add a way to use proportional controls to activate toggles with a threshold value also stick buttons???????????
    public enum Button {
        A, B, X, Y,
        D_UP, D_DOWN, D_LEFT, D_RIGHT,
        BUMPER_LEFT, BUMPER_RIGHT
    }

    public enum GamepadId {
        User1,
        User2;

        public GamepadInfo getGamepad () {
            switch (this) {
                case User1:
                    return gamepad1;
                case User2:
                    return gamepad2;
                default:
                    return gamepad1;
            }
        }
    }

    class GamepadInfo {
        public Gamepad gamepad;
        public Gamepad previous;

        public boolean firstPress;

        public boolean getToggled (Button button, Boolean current) {
            switch (button) {
                case A:
                    return (current ? gamepad.a : previous.a);
                case B:
                    return (current ? gamepad.b : previous.b);
                case X:
                    return (current ? gamepad.x : previous.x);
                case Y:
                    return (current ? gamepad.y : previous.y);
                case D_UP:
                    return (current ? gamepad.dpad_up : previous.dpad_up);
                case D_DOWN:
                    return (current ? gamepad.dpad_down : previous.dpad_down);
                case D_LEFT:
                    return (current ? gamepad.dpad_left : previous.dpad_left);
                case D_RIGHT:
                    return (current ? gamepad.dpad_right : previous.dpad_right);
                case BUMPER_LEFT:
                    return (current ? gamepad.left_bumper : previous.left_bumper);
                case BUMPER_RIGHT:
                    return (current ? gamepad.right_bumper : previous.right_bumper);
                default:
                    return false;
            }
        }
    }

    static GamepadInfo gamepad1;
    static GamepadInfo gamepad2;

    GamepadId gamepad;
    Button toggleButton;

    public Toggle(Button button, GamepadId gamepad) {
        toggleButton = button;
        this.gamepad = gamepad;
    }
    public Toggle(){

    }

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
        }
        if (gamepad.getGamepad().getToggled(toggleButton, true) && !(gamepad.getGamepad().getToggled(toggleButton, false))) {
            toggled();
        }
    }

    abstract void toggled ();

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

    @Override
    public boolean equals(Object o) {
        Toggle other = (Toggle)o;
        return (toggleButton == other.toggleButton) && (gamepad == other.gamepad);
    }
}
