package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.curve;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class logger {
    enum opMode {
        TELEOP,
        AUTO
    }

    private static final String COMMA_DELIMITER = ",";
    private static final String SEMICOLON_DELIMITER = ";";
    static String FILENAME = "log_";
    static String PATH = String.format("%s/FIRST/custom_log/%s.csv", Environment.getExternalStorageDirectory().getPath(), FILENAME);
    public static FileWriter fw;
    static boolean init = false;

    static {
        try {
            fw = new FileWriter(PATH, false);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    static String SEPARATOR = ": ";

    public static void init(){
        init = true;
        Timestamp timestamp = new Timestamp(System.currentTimeMillis());
        FILENAME = FILENAME.concat(timestamp.toString());
        PATH = String.format("%s/FIRST/custom_log/%s.csv", Environment.getExternalStorageDirectory().getPath(), FILENAME);

        try {
            File directory = new File(String.format("%s/FIRST/custom_log", Environment.getExternalStorageDirectory().getPath()));
            if (!directory.exists()) {
                directory.mkdirs();
            }
            fw = new FileWriter(String.format("%s/FIRST/custom_log/%s.csv", Environment.getExternalStorageDirectory().getPath(), FILENAME), false);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void init(String filename){
        init = true;
        Timestamp timestamp = new Timestamp(System.currentTimeMillis());
        FILENAME = filename.concat(timestamp.toString());
        PATH = String.format("%s/FIRST/custom_log/%s.csv", Environment.getExternalStorageDirectory().getPath(), FILENAME);

        try {
            File directory = new File(String.format("%s/FIRST/custom_log", Environment.getExternalStorageDirectory().getPath()));
            if (!directory.exists()) {
                directory.mkdirs();
            }
            fw = new FileWriter(PATH, false);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void write(String content) {
        if (!init)
            return;
        Timestamp timestamp = new Timestamp(System.currentTimeMillis());
        try {
            fw.write(content + " " + timestamp);
            fw.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static void writeLn(String content) {
        if (!init)
            return;
        Timestamp timestamp = new Timestamp(System.currentTimeMillis());
        try {
            fw.write("\n" + content + " " + timestamp);
            fw.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static void addData(String caption, Object value) {
        if (!init)
            return;
        Timestamp timestamp = new Timestamp(System.currentTimeMillis());
        try {
            fw.write("\n" + caption + SEPARATOR + value.toString() + " " + timestamp);
            fw.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    public static void close(){
        try {
            fw.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
