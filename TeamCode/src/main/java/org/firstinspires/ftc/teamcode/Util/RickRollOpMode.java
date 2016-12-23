package org.firstinspires.ftc.teamcode.Util;

import android.app.Application;
import android.content.Context;
import android.media.AudioManager;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Environment;
import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.FileInputStream;
import java.io.IOException;

/**
 * Created by hsunx on 11/5/2016.
 */
@Autonomous (name = "rickroll", group = "troll")
public class RickRollOpMode extends OpMode {
    MediaPlayer player;
    String filePath;
    @Override
    public void init() {
        filePath = Environment.getExternalStorageDirectory().getPath() + "/FIRST/rickroll.mp3";
        RobotLog.d(filePath);
    }

    @Override
    public void start() {
        super.start();


    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        super.stop();
    }
}
