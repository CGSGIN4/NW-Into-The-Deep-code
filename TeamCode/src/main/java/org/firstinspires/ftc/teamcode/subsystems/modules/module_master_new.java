package org.firstinspires.ftc.teamcode.subsystems.modules;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.PITCH_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_ROTATION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_YELLOW1;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_YELLOW2;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_YELLOW3;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_ROTATION_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_ROTATION_LIFT;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

/* contains sample action could be done in auto. Dalshe ebites sami */
public class module_master_new {
    /* add actions here following byte flags conception */
    public enum action{;
        public static final int CLAW_CLOSE = 1;
        public static final int CLAW_OPEN = 2;
        public static final int SET_ROTATION_LIFT = 3;
        public static final int SET_ROTATION_FRONT = 4;
        public static final int SET_EXTENSION_LIFT = 5;
        public static final int SET_EXTENSION_CLOSED = 6;
        public static final int SET_EXTENSION_LIMIT = 7;
        public static final int PITCH_DOWN = 8;
        public static final int PITCH_FRONT = 9;
        public static final int SET_EXTENSION_YELLOW1 = 10;
        public static final int SET_EXTENSION_YELLOW2 = 11;
        public static final int SET_EXTENSION_YELLOW3 = 12;
        public static final int SET_EXTENSION_CHAMBER = 13;
        public static final int SET_ROTATION_CHAMBER = 14;
    }

    enum cmdType{
        EXTENSION,
        ROTATION
    }

    static cmdType lastCmdType = null;
    public static differential differential;
    public static arm arm;
    public static Queue<Integer> commandQueue = new LinkedList<Integer>();
    static Integer[] rotationActions = {3, 4};
    static Integer[] extensionActions = {5, 6, 7, 10, 11, 12, 13, 14};

    public static void init(HardwareMap HM){
        arm = new arm(HM);
        differential = new differential(HM);
        commandQueue.clear();
    }

    public static void schedule(int action){
        commandQueue.add(action);
    }
    public static void doAction(int action){
        switch (action)
        {
            case CLAW_CLOSE:
                differential.closeClaw();
                break;
            case CLAW_OPEN:
                differential.openClaw();
                break;
            case SET_ROTATION_LIFT:
                arm.setRotation(org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT);
                break;
            case SET_ROTATION_FRONT:
                arm.setRotation(org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT);
                break;
            case SET_EXTENSION_LIFT:
                arm.setExtension(org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED);
                break;
            case SET_EXTENSION_CLOSED:
                arm.setExtension(org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED);
                break;
            case SET_EXTENSION_LIMIT:
                arm.setExtension(org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.FRONTAL_EXTENSION);
                break;
            case SET_EXTENSION_YELLOW1:
                arm.setExtension(org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.YELLOW_1);
                break;
            case SET_EXTENSION_YELLOW2:
                arm.setExtension(org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.YELLOW_2);
                break;
            case SET_EXTENSION_YELLOW3:
                arm.setExtension(org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.YELLOW_3);
                break;
            case SET_EXTENSION_CHAMBER:
                arm.setExtension(org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.HIGH_CHAMBER);
                break;
            case SET_ROTATION_CHAMBER:
                arm.setRotation(org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.CHAMBER);
                break;
            case PITCH_DOWN:
                differential.pitchDown();
                break;
            case PITCH_FRONT:
                differential.pitchForward();
                break;
            default:
                break;
        }
    }

    public static void update(MultipleTelemetry telemetry){
        arm.update(telemetry);
        telemetry.addData("current cmd type", lastCmdType == null ? "null" : lastCmdType.toString());
        telemetry.addData("current ext target", arm.extensionState);
        telemetry.addData("current rot target", arm.rotationState);
        telemetry.addData("ext reached", arm.extensionReached());
        telemetry.addData("queue", commandQueue.size());
        telemetry.update();

        if (lastCmdType == null || !commandQueue.isEmpty() && lastCmdType == cmdType.EXTENSION && arm.extensionReached()) {
            int action = commandQueue.poll();
            doAction(action);
            lastCmdType = (Arrays.asList(rotationActions).contains(action) ? cmdType.ROTATION : cmdType.EXTENSION);
        }
        else
        if (lastCmdType == null || !commandQueue.isEmpty() && lastCmdType == cmdType.ROTATION && arm.rotationReached()) {
            int action = commandQueue.poll();
            doAction(action);
            lastCmdType = (Arrays.asList(rotationActions).contains(action) ? cmdType.ROTATION : cmdType.EXTENSION);
        }
    }

    public static void stop(MultipleTelemetry telemetry){
        arm.stop();
    }
}
