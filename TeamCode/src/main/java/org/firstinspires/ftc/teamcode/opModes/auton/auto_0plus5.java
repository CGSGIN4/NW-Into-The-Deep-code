package org.firstinspires.ftc.teamcode.opModes.auton;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_DOBOR;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_YELLOW2;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_PREASCEND;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.data.transfer;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.utils.logger;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.IOException;
import java.util.function.BooleanSupplier;

@Autonomous
public class auto_0plus5 extends LinearOpMode {
    Robot robot;
    path_follower path_follower;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //logger.init();
        robot = new Robot(hardwareMap);
        robot.init();
        parser parser = new parser("b19dota");
        dataStorage.init(robot.drive, telemetry, this);
        path_follower = new path_follower(robot.drivetrain);
        robot.drive.setPoseEstimate(new Pose2d(39.9, 64.93, Math.PI));

        curve[] curves;
        try {
            curves = parser.getCurves();
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }
        module_master.differential.pitchUp();
        module_master.differential.rollDefault();
        module_master.differential.closeClaw();
        module_master.differential.update();

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

        while (opModeIsActive()) {
            prepareToScoreHighBasket();
            path_follower.goToPos(curves[0].getPoint(1).getX(), curves[0].getPoint(1).getY(), -Math.PI * 3 / 4);
            //path_follower.followTrajectoryBreak(curves[0], -Math.PI * 3 / 4);
            scoreHighBasket();

            path_follower.followTrajectory(curves[1], Math.PI - Math.toRadians(5.1), new double[]{0.3, 0.7}, new int[]{SET_ROTATION_FRONT, SET_EXTENSION_DOBOR});
            module_master.differential.pitchForward();
            path_follower.goToPos(-21.6, 61.7, Math.PI - Math.toRadians(5.3));

            waitArmExtension();
            module_master.differential.setPitch(23);
            module_master.differential.update();
            delay(300);
            module_master.differential.closeClaw();
            delay(200);

            module_master.differential.pitchForward();

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);

            path_follower.followTrajectoryBackwardsPercentage(curves[3], 70, new double[]{0.4, 0.4}, new int[]{PITCH_DOWN, SET_EXTENSION_LIFT});
            path_follower.goToPosWithArmToBasket(52.4, 50.4, -Math.PI * 3 / 4);
            waitArmExtension();
            scoreHighBasket(); /* score dobor */

            path_follower.goToPos(47.6, 48.9, -Math.PI / 2 - Math.toRadians(1.2)); /* sample 1 */
            //path_follower.goToPosUnsafe(51.4, 51.4, -Math.PI / 2 - Math.toRadians(11.5)); /* sample 1 */
            waitArmRotation();
            module_master.differential.setPitch(25);
            module_master.differential.update();
            setExtensionAndWait(arm.extension.YELLOW_1);
            takeSample(false);
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchForward();
            path_follower.goToPosWithArmToBasket(52.4, 50.4, -Math.PI * 3 / 4);
            module_master.differential.pitchHalfDown();

            setExtensionAndWait(arm.extension.EXTENDED);
            scoreHighBasket();

            path_follower.goToPos(57.7, 47.6, -Math.PI / 2); /* sample 2 */
            waitArmRotation();
            module_master.differential.setPitch(25);
            module_master.differential.update();
            module_master.arm.setExtension(arm.extension.YELLOW_2);
            waitArmExtension();

            takeSample(false);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchForward();
            path_follower.goToPosWithArmToBasket(50.6, 50.6, -Math.PI * 3 / 4);
            module_master.differential.pitchHalfDown();

            setExtensionAndWait(arm.extension.EXTENDED);
            scoreHighBasket();

            path_follower.goToPos(59.5, 44.3, -Math.PI / 2 + Math.toRadians(20.8));
            waitArmRotation();

            /* intaking yellow 3 */
            module_master.differential.update();
            setExtensionAndWait(arm.extension.YELLOW_3);
            takeSample(true);
            module_master.differential.rollDefault();

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchForward();
            path_follower.goToPosWithArmToBasket(51.7, 46.2, -Math.PI * 3 / 4);
            module_master.differential.pitchHalfDown();

            setExtensionAndWait(arm.extension.EXTENDED);
            scoreHighBasket();

            path_follower.followTrajectory(curves[4], -Math.PI, new double[]{0.2, 0.2, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            path_follower.goToPosVeryUnsafe(21, 6, -Math.PI);
            module_master.arm.setRotation(arm.rotation.CHAMBER);
            while (opModeIsActive()) {
                module_master.arm.update();
                module_master.arm.manuallyExtend(0);
            }
            robot.stop();
            //module_master.stop(dataStorage.telemetry);
            transfer.angle = robot.drive.getPoseEstimate().getHeading();
            //transfer.armExtensionPos = module_master.arm.extensionMotor.getCurrentPosition();

            packet = new TelemetryPacket();
            fieldOverlay = packet.fieldOverlay();
            path_follower.painter.prepare(packet, fieldOverlay);
            delay(10000);

            for (curve traj : curves) {
                if (traj != null)
                    path_follower.painter.drawPolyLine(traj.points, "green");

                path_follower.painter.drawPolyLine(dataStorage.poseHistory.toArray(new Vector2d[0]), "blue");
            }

            path_follower.dashboard.sendTelemetryPacket(packet);

            //timer.reset();
            //while(timer.seconds() < 2 && opModeIsActive());
            break;
        }
        robot.stop();
        module_master.stop(dataStorage.telemetry);
    }

    private void waitForCondition(BooleanSupplier condition) {
        timer.reset();
        while (!condition.getAsBoolean() && opModeIsActive() && timer.milliseconds() < 5000) {
            module_master.update(dataStorage.telemetry);
            dataStorage.updateData();
        }
    }

    private void delay(long milliseconds) {
        timer.reset();
        while (timer.milliseconds() < milliseconds && opModeIsActive()) {
            module_master.update(dataStorage.telemetry);
            dataStorage.updateData();
        }
    }

    private void waitArmExtension(){
        waitForCondition(() -> module_master.arm.extensionReached());
    }

    private void waitArmRotation(){
        waitForCondition(() -> module_master.arm.rotationReached());
    }

    /** PITCH DOWN, CLOSE CLAW, PITCH FORWARD **/
    private void takeSample(boolean third){
        if (!third) {
            module_master.differential.setPitch(25);
            module_master.differential.update();
        }
        else {
            module_master.differential.setPitch(22);
            module_master.differential.setRoll(23);
            module_master.differential.update();
            delay(200);
        }

        module_master.differential.closeClaw();
        delay(200);

        module_master.differential.pitchForward();
    }

    /** SET DIFFY PITCH DOWN AND ARM ROTATED TO LIFT **/
    private void prepareToScoreHighBasket(){
        module_master.differential.pitchDown();
        module_master.arm.setRotation(arm.rotation.LIFT);
        module_master.arm.setExtension(arm.extension.EXTENDED);
    }

    private void setExtensionAndWait(arm.extension extension){
        module_master.arm.setExtension(extension);
        waitArmExtension();
    }

    /** RAISE ELEVATOR, SCORE SAMPLE, FOLD EXTENSION **/
    private void scoreHighBasket(){
        waitArmRotation();
        waitArmExtension();

        /* SCORE SAMPLE */
        module_master.differential.pitchScoringBasket();
        delay(300);

        //waitArmRotation();
        module_master.differential.openClaw();
        delay(300);

        module_master.differential.pitchHalfDown();
        delay(100);

        /* FOLD */
        module_master.arm.setExtension(arm.extension.CLOSED);
        module_master.arm.setRotation(arm.rotation.FRONT);
    }
}