package org.lasarobotics.library.util;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.util.RobotLog;

import org.lasarobotics.library.android.Util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Type;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;


import com.google.common.io.Files;

/**
 * Simple logger that can write to multiple file formats for analyzing robot telemetry
 */
public class Log {
    private String fileDirectory;
    private String fileName;
    private Timers timers;
    private ArrayList<LogData> logEntries;

    public Log(String fileDirectory, String fileName) {
        this.fileDirectory = fileDirectory;
        this.fileName = fileName;
        timers = new Timers();
        timers.startClock("log");
        logEntries = new ArrayList<>();
    }

    /**
     * Add an entry to the log
     *
     * @param tag  Tag associated with the data point
     * @param data Data for log entry
     */
    public void add(String tag, String data) {
        logEntries.add(new LogData(timers.getClockValue("log"), tag, data));
        RobotLog.i(data);
        RobotLog.i(tag);
    }

    /**
     * Saves a log to fileName specified and with format specified
     * <p/>
     * This function will not overwrite an existing log file, but append ".1", ".2", etc. if it already exists
     *
     * @param fileType Format to write file in
     */

    public void saveAs(FileType fileType) {
        try {
            //Use correct filename for requested file type
            File f = Util.createFileOnDevice("FIRST/", fileName + "." + fileType.toString(), false);
            StringBuilder out = new StringBuilder();
            switch (fileType) {
                case JSON:
                    Type logDataList = new TypeToken<List<LogData>>() {
                    }.getType();
                    Gson g = new Gson();
                    out.append(g.toJson(logEntries, logDataList));
                    break;
                case CSV:
                    out.append("time,tag,data\n");
                    for (LogData l : logEntries) {
                        out.append("\"" + l.time + "\"," + "\"" + l.tag + "\"," + "\"" + l.data + "\"\n");
                    }
                    break;
                case TEXT:
                    for (LogData l : logEntries) {
                        out.append(l.time + ":[" + l.tag + "]" + l.data + "\n");
                    }
                    break;
            }
            RobotLog.i(out.toString());
            BufferedWriter writer = new BufferedWriter(new FileWriter(f));
            writer.write(out.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String getFileDirectory() {
        return fileDirectory;
    }

    public void setFileDirectory(String fileDirectory) {
        this.fileDirectory = fileDirectory;
    }

    public String getFileName() {
        return fileName;
    }

    public void setFileName(String fileName) {
        this.fileName = fileName;
    }

    public enum FileType {
        JSON("json"),
        CSV("csv"),
        TEXT("txt");

        private String fileType;

        FileType(String deptName) {
            this.fileType = deptName;
        }

        @Override
        public String toString() {
            return this.fileType;
        }
    }

    private class LogData {
        long time;
        String tag;
        String data;

        public LogData(long time, String tag, String data) {
            this.time = time;
            this.tag = tag;
            this.data = data;
        }
    }
}
