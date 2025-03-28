package org.firstinspires.ftc.teamcode.data;

import static org.firstinspires.ftc.teamcode.math.normalizeAngle.normalizeAngle;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.painter;
import org.firstinspires.ftc.teamcode.utils.ring_buffer;

import java.util.ArrayList;
import java.util.List;

public enum dataStorage {;
    public static SampleMecanumDrive drive;
    public static Vector2d RobotPose;
    /* stores X coordinate of the robot */
    public static double RobotWorldX;
    /**
     * stores Y coordinate of the robot
     */
    public static double RobotWorldY;
    /**
     * stores heading of the robot from -PI to PI
     */
    public static double RobotWorldHeading;
    /* stores robot velocity */
    public static Vector2d RobotVelocity;

    public static double OldRobotWorldHeading;
    /* stores the acceleration with which robot breaks */
    public static double BrakeAccel = 104; //160

    public static Telemetry DSTelemetry;
    public static Telemetry DashTelemetry;
    public static MultipleTelemetry telemetry;

    public static FtcDashboard dashboard;
    public static painter painter;

    public static LinearOpMode OpMode;
    public static List<Vector2d> poseHistory = new ArrayList<>();

    public static ElapsedTime timer =new ElapsedTime();
    public static ElapsedTime relocation_timer =new ElapsedTime();

    public static int iter;
    public static Vector2d relocation = new Vector2d(0, 0);

    public static ring_buffer<Vector2d> lastPoses = new ring_buffer<>(3);
    public static boolean opModeIsAutonomous = true;
    public static double photoVel = 0;

    public static void updateData()
    {
        drive.update();
        Pose2d pose = drive.getPoseEstimate();
        lastPoses.put(pose.vec());
        //lastPoses.output();

        iter++;
        relocation = relocation.plus(new Vector2d(pose.getX() - RobotWorldX, pose.getY() - RobotWorldY));
        if (iter == 2)
        {
            RobotVelocity = relocation.div(relocation_timer.seconds());
            relocation = new Vector2d(0, 0);
            iter = 0;
            relocation_timer.reset();
        }

        BrakeAccel = calculateBrakeAccel(RobotVelocity.norm());

        RobotWorldX = pose.getX();
        RobotWorldY = pose.getY();

        OldRobotWorldHeading = RobotWorldHeading;

        RobotWorldHeading = normalizeAngle(pose.getHeading());

        RobotPose = new Vector2d(RobotWorldX, RobotWorldY);
        poseHistory.add(RobotPose);

        timer.reset();
        telemetry.addData("ROBOT X", RobotWorldX);
        telemetry.addData("ROBOT Y", RobotWorldY);
        telemetry.addData("ROBOT HEADING", RobotWorldHeading);
        telemetry.addData("photoVel", photoVel);
        //telemetry.addData("heading", RobotWorldHeading);
        telemetry.update();
        //RobotVelocity = new Vector2d(drive.getPoseVelocity().getX(), drive.getPoseVelocity().getY());
    }

    public static void init(SampleMecanumDrive drive, Telemetry telemetryy, LinearOpMode linearOpMode){
        dataStorage.drive = drive;
        Pose2d pose = drive.getPoseEstimate();
        RobotWorldX = pose.getX();
        RobotWorldY = pose.getY();
        RobotWorldHeading = OldRobotWorldHeading = pose.getHeading();
        RobotVelocity = new Vector2d(0, 0);
        BrakeAccel = 0;
        DSTelemetry = telemetryy;
        dashboard = FtcDashboard.getInstance();
        DashTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(DSTelemetry, DashTelemetry);
        OpMode = linearOpMode;
        poseHistory.clear();
        iter = 0;
        relocation_timer.reset();
        module_master.init(linearOpMode.hardwareMap);
    }

    private static double calculateBrakeAccel(double v){
        double v2 = v * v;
        double v3 = v2 * v;
        //return 0.0006579582449 * v3 - 0.1265954365481 * v2 + 8.8226496330171 * v - 88.7730817754054;
        return -0.001485206174316 * v3 + 0.229578434273549 * v2 - 10.423935051483568 * v + 236.497982889413834;
    }
}
