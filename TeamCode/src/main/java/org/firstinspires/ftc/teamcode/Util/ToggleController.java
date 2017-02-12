package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;

/**
 * Created by hsunx on 12/3/2016.
 */

public class ToggleController {
    static ArrayList<Toggle> toggles;

    public static void clearToggles () {
        toggles.clear();
    }

    public static void addToggle (Toggle toggle) {
        if (toggles == null) {
            toggles = new ArrayList<>();
        }
        toggles.add(toggle);
    }

    public static void removeToggle (Toggle toggle) {
        if (toggles == null) {
            return;
        }
        try {
            toggles.remove(toggle);
        }
        catch (Exception e) {
            RobotLog.e("Toggle you are trying to remove does not exist.");
        }
    }

    public static void update () {
        for (Toggle toggle : toggles) {
            toggle.update();
        }
        Toggle.updatesFinished();
    }

}
