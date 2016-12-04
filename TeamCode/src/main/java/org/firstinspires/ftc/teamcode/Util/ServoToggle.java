package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Josh on 12/3/2016.
 */

public class ServoToggle extends Toggle {

    double[] positions;
    int index = 0;

    public void setPositions (double[] positions) {
        // Make sure the positions are valid
        for (int i = 0; i < positions.length; i++) {
            positions[i] = Range.clip(positions[i], 0, 1);
        }

        this.positions = positions;
    }

    @Override
    public void update() {
        super.update();


    }
}
