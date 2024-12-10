package org.firstinspires.ftc.teamcode.opModes.deprecated;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.IOException;
@Disabled
@TeleOp(group = "Deprecated")
public class raskatka extends LinearOpMode {

    Robot robot;
    org.firstinspires.ftc.teamcode.subsystems.path_follower path_follower;
    ElapsedTime timer = new ElapsedTime(), main_timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.init();
        parser parser = new parser();
        dataStorage.init(robot.drive, telemetry, this);
        path_follower = new path_follower(robot.drivetrain);
        robot.drive.setPoseEstimate(new Pose2d(-0.374, -0.479   , 0));

        curve[] curves;
        try {
            curves = parser.getCurves();
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }

        waitForStart();
        main_timer.reset();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        path_follower.painter.prepare(packet, fieldOverlay);

        for (curve traj : curves) {
            if (traj != null)
                path_follower.painter.drawPolyLine(traj.points, "green");
        }

        path_follower.dashboard.sendTelemetryPacket(packet);

        int index = 0;
        //_________________________________________
        /*
        for (curve traj : Trajectories) {
                if (index < Trajectories.length - 1)
                    path_follower.followTrajectory(traj);
                index++;
         */

        while (opModeIsActive()) {
            timer.reset();

            path_follower.followTrajectory(curves[0]);
            path_follower.followTrajectory(curves[1]);
            path_follower.followTrajectory(curves[2]);
            path_follower.followTrajectory(curves[3]);
            path_follower.followTrajectory(curves[4]);
            path_follower.followTrajectory(curves[5]);
            path_follower.followTrajectory(curves[6]);
            path_follower.followTrajectoryBreak(curves[7]);
            if (gamepad1.a) break;
            packet = new TelemetryPacket();
            packet.put("full_time", main_timer.seconds());
            path_follower.dashboard.sendTelemetryPacket(packet);
            dataStorage.DSTelemetry.addData("time", timer.milliseconds());
            dataStorage.DSTelemetry.update();
            timer.reset();
            while(timer.seconds() < 2 && opModeIsActive());
        }

            //_______________________________________

            //path_follower.followTrajectoryBreak(curves[curves.length - 1]);

            robot.stop();


            packet = new TelemetryPacket();
            fieldOverlay = packet.fieldOverlay();
            path_follower.painter.prepare(packet, fieldOverlay);

            for (curve traj : curves) {
                if (traj != null)
                    path_follower.painter.drawPolyLine(traj.points, "green");

                path_follower.painter.drawPolyLine(dataStorage.poseHistory.toArray(new Vector2d[0]), "blue");
            }
        packet.put("full_time", main_timer.seconds());

        path_follower.dashboard.sendTelemetryPacket(packet);

            //timer.reset();
            //while(timer.seconds() < 2 && opModeIsActive());

            //timer.reset();
            //while (timer.seconds() < 200 && opModeIsActive()) ;
    }
}
