package org.firstinspires.ftc.teamcode.subsystems.modules;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_LIFT;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

/* contains sample action could be done in auto. Dalshe ebites sami */
public class module_master {
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
        public static final int SET_EXTENSION_CHAMBER = 10;
        public static final int SET_ROTATION_CHAMBER = 11;

    }

    enum cmdType{
        EXTENSION,
        ROTATION
    }

    public static differential differential;
    public static arm arm;

    public static void init(HardwareMap HM){
        arm = new arm(HM);
        differential = new differential(HM);
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
    }

    public static void stop(MultipleTelemetry telemetry){
        arm.stop();
    }
}
