package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_LIFT;

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
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;
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
        robot.drive.setPoseEstimate(new Pose2d(39.9, 64.93, Math.PI));

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

            module_master.differential.pitchUp();
            module_master.differential.rollDefault();
            module_master.differential.closeClaw();
            path_follower.followTrajectoryBreak(curves[0], -Math.PI * 3 / 4, new double[]{0.1}, new int[]{SET_ROTATION_LIFT});
            dataStorage.DSTelemetry.addData("time", timer.milliseconds());
            dataStorage.DSTelemetry.update();

            timer.reset();

            /* PRELOAD */
            while (!module_master.arm.rotationReached() && opModeIsActive())
                module_master.update();

            module_master.doAction(SET_EXTENSION_LIFT);
            while (!module_master.arm.extensionReached() && opModeIsActive())
                module_master.update();

            module_master.differential.setPitch(150);
            timer.reset();
            while (timer.milliseconds() < 300)
                module_master.update();

            module_master.differential.openClaw();
            timer.reset();
            while (timer.milliseconds() < 200)
                module_master.update();

            module_master.differential.pitchForward();
            timer.reset();
            while (timer.milliseconds() < 300)
                module_master.update();

            module_master.doAction(SET_EXTENSION_CLOSED);
            while (!module_master.arm.extensionReached() && opModeIsActive())
                module_master.update();

            module_master.doAction(SET_ROTATION_FRONT);
            while (!module_master.arm.rotationReached() && opModeIsActive())
                module_master.update();

            /* 1ST YELLOW */
            path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY, -Math.PI / 2 + Math.PI / 7);
            timer.reset();

            module_master.arm.setExtension(arm.extension.YELLOW_1);
            while (!module_master.arm.extensionReached() && opModeIsActive())
                module_master.update();

            module_master.differential.pitchDown();
            timer.reset();
            while (timer.milliseconds() < 200)
                module_master.update();

            module_master.differential.closeClaw();
            timer.reset();
            while (timer.milliseconds() < 200)
                module_master.update();

            module_master.differential.pitchForward();
            timer.reset();
            while (timer.milliseconds() < 200)
                module_master.update();

            module_master.arm.setExtension(arm.extension.CLOSED);
            while (!module_master.arm.extensionReached() && opModeIsActive())
                module_master.update();

            /* SCORE 1ST YELLOW */
            module_master.doAction(SET_ROTATION_LIFT);
            while (!module_master.arm.rotationReached() && opModeIsActive())
                module_master.update();

            module_master.doAction(SET_EXTENSION_LIFT);
            while (!module_master.arm.extensionReached() && opModeIsActive())
                module_master.update();

            module_master.differential.setPitch(150);
            timer.reset();
            while (timer.milliseconds() < 300)
                module_master.update();

            module_master.differential.openClaw();
            timer.reset();
            while (timer.milliseconds() < 200)
                module_master.update();

            module_master.differential.pitchForward();
            timer.reset();
            while (timer.milliseconds() < 300)
                module_master.update();

            module_master.doAction(SET_EXTENSION_CLOSED);
            while (!module_master.arm.extensionReached() && opModeIsActive())
                module_master.update();

            module_master.doAction(SET_ROTATION_FRONT);
            while (!module_master.arm.rotationReached() && opModeIsActive())
                module_master.update();

            /* 2ND YELLOW */

            while(timer.seconds() < 2);
            path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY, -Math.PI * 3 / 4);
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