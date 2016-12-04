package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Josh on 12/3/2016.
 */

public class ServoToggle extends Toggle {

    double[] positions;
    int index = 0;

    Servo servo;

    public ServoToggle (Button button, Servo servo, double[] positions, GamepadId gamepad) {

        super(button, gamepad);
        // Make sure the positions are valid
        for (int i = 0; i < positions.length; i++) {
            positions[i] = Range.clip(positions[i], 0, 1);
        }

        this.positions = positions;
        this.servo = servo;

    }

    @Override
    void toggled() {
        servo.setPosition(positions[index]);
        index++;
    }
}
