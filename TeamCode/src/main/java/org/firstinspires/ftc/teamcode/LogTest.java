package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.lasarobotics.library.util.Log;

/**
 * Created by hsunx on 12/30/2016.
 */
@Autonomous(name = "log test", group = "fart")
public class LogTest extends OpMode {
    Log log;

    @Override
    public void init() {
        log = new Log("FIRST", "testlog");
    }

    @Override
    public void loop() {
        log.add("test", "test info 1");
        log.add("testing", "123");
    }

    @Override
    public void stop() {
        log.saveAs(Log.FileType.TEXT);
        super.stop();
    }
}
