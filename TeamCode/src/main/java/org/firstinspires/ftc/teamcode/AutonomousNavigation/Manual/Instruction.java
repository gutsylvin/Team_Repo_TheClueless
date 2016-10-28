package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import java.util.Map;

/**
 * Created by Josh on 10/27/2016.
 */

public interface Instruction {
    void Loop ();
    boolean Finished ();
    void Init ();
    void FromMap(Map<String, String> map);
}
