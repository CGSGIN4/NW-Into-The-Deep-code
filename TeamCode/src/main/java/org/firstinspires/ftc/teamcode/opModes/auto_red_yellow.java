package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_OPEN;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.IOException;

@Autonomous
public class auto_red_yellow extends LinearOpMode {

    Robot robot;
    path_follower path_follower;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.init();
        parser parser = new parser("ITD_RED_YELLOW");
        dataStorage.init(robot.drive, telemetry, this);
        path_follower = new path_follower(robot.drivetrain);
        robot.drive.setPoseEstimate(new Pose2d(11.901, 61.762, Math.PI / 2));

        curve[] curves;
        try {
            curves = parser.getCurves();
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }

        waitForStart();

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

            path_follower.followTrajectoryBreak(curves[0], Math.PI / 2);
            timer.reset();
            while (timer.milliseconds() < 1000);
            path_follower.followTrajectoryBreak(curves[1], 0);
            timer.reset();
            while (timer.milliseconds() < 1000);
            path_follower.followTrajectoryBreak(curves[2], Math.PI / 4, new double[]{0.2, 0.7}, new int[]{CLAW_OPEN, CLAW_CLOSE});
            timer.reset();
            while (timer.milliseconds() < 1000);
            path_follower.followTrajectoryBreak(curves[3], 0);
            timer.reset();
            while (timer.milliseconds() < 1000);
            path_follower.followTrajectoryBreak(curves[4], Math.PI / 4);
            dataStorage.DSTelemetry.addData("time", timer.milliseconds());
            dataStorage.DSTelemetry.update();

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

            path_follower.dashboard.sendTelemetryPacket(packet);

            //timer.reset();
            //while(timer.seconds() < 2 && opModeIsActive());

            timer.reset();
            while (timer.seconds() < 200 && opModeIsActive()) ;
        }
    }
}