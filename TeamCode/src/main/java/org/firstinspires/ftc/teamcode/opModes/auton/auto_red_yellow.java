package org.firstinspires.ftc.teamcode.opModes.auton;

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
import java.util.function.BooleanSupplier;

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

        /* INIT */

        module_master.differential.pitchUp();
        module_master.differential.rollDefault();
        module_master.differential.closeClaw();
        module_master.differential.update();

        while (opModeIsActive()) {
            /* PRELOAD */
            //prepareToScoreHighBasket();
            path_follower.followTrajectoryBreak(curves[0], -Math.PI * 3 / 4);
            //scoreHighBasket();

            /* ---------1ST YELLOW--------- */
            path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY, -Math.PI / 2 - Math.toRadians(16));
            //waitArmRotation();

            //module_master.differential.pitchHalfDown();
            setExtensionAndWait(arm.extension.YELLOW_1);
            delay(1000);

            //takeSample();

            setExtensionAndWait(arm.extension.CLOSED);
            //prepareToScoreHighBasket();
            path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY, -Math.PI * 3 / 4);
            //scoreHighBasket();

            /* ---------2ND YELLOW--------- */
            path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY, -Math.PI / 2 + Math.toRadians(12));
            //waitArmRotation();

            //module_master.differential.pitchHalfDown();
            setExtensionAndWait(arm.extension.YELLOW_1);
            delay(1000);

            //takeSample();

            setExtensionAndWait(arm.extension.CLOSED);
            //prepareToScoreHighBasket();
            path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY, -Math.PI * 3 / 4);
            //scoreHighBasket();

            /* ---------3RD YELLOW--------- */
            //module_master.differential.rollHalfRight();
            path_follower.goToPos(58, 49.6, -Math.PI / 2 + Math.toRadians(18));
            //waitArmRotation();

            setExtensionAndWait(arm.extension.YELLOW_1);
            delay(1000);

            //takeSample();

            module_master.arm.setExtension(arm.extension.CLOSED);
            path_follower.goToPos(53.5, 53.5, -Math.PI * 3 / 4);
            waitArmExtension();
            //prepareToScoreHighBasket();
            //scoreHighBasket();

            //waitArmRotation();

            /* 4TH YELLOW */
            path_follower.followTrajectory(curves[1], Math.PI);
            path_follower.followTrajectoryBreak(curves[2], Math.PI);
            module_master.arm.setExtension(arm.extension.YELLOW_1);
            waitArmExtension();
            delay(1000);
            module_master.arm.setExtension(arm.extension.CLOSED);
            path_follower.followTrajectoryBreak(curves[3], -Math.PI * 3 / 4);
            waitArmExtension();

            robot.stop();
            module_master.stop(dataStorage.telemetry);

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

    private void waitForCondition(BooleanSupplier condition) {
        while (!condition.getAsBoolean() && opModeIsActive()) {
            module_master.update(dataStorage.telemetry);
        }
    }

    private void delay(long milliseconds) {
        timer.reset();
        while (timer.milliseconds() < milliseconds && opModeIsActive()) {
            module_master.update(dataStorage.telemetry);
        }
    }

    private void waitArmExtension(){
        waitForCondition(() -> module_master.arm.extensionReached());
    }

    private void waitArmRotation(){
        waitForCondition(() -> module_master.arm.rotationReached());
    }

    /** PITCH DOWN, CLOSE CLAW, PITCH FORWARD **/
    private void takeSample(){
        module_master.differential.pitchDown();
        delay(200);

        module_master.differential.closeClaw();
        delay(200);

        module_master.differential.pitchForward();
    }

    /** SET DIFFY PITCH DOWN AND ARM ROTATED TO LIFT **/
    private void prepareToScoreHighBasket(){
        module_master.doAction(SET_ROTATION_LIFT);
        module_master.differential.pitchDown();
    }

    private void setExtensionAndWait(arm.extension extension){
        module_master.arm.setExtension(extension);
        waitArmExtension();
    }

    /** RAISE ELEVATOR, SCORE SAMPLE, FOLD EXTENSION **/
    private void scoreHighBasket(){
        waitArmRotation();

        /* RAISE ELEVATOR */
        module_master.doAction(SET_EXTENSION_LIFT);
        waitArmExtension();

        /* SCORE SAMPLE */
        module_master.differential.pitchScoringBasket();
        delay(300);

        module_master.differential.openClaw();
        delay(300);

        module_master.differential.pitchHalfDown();
        delay(300);

        /* FOLD */
        module_master.doAction(SET_EXTENSION_CLOSED);
        waitArmExtension();

        module_master.doAction(SET_ROTATION_FRONT);
    }
}