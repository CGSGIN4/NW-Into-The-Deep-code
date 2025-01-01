package org.firstinspires.ftc.teamcode.opModes.auton;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.PITCH_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_YELLOW1;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_YELLOW2;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_EXTENSION_YELLOW3;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_ROTATION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_ROTATION_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new.action.SET_ROTATION_LIFT;

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
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master_new;
import org.firstinspires.ftc.teamcode.subsystems.path_follower_new;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.IOException;
import java.util.function.BooleanSupplier;

@Autonomous
public class auto_new extends LinearOpMode {
    Robot robot;
    path_follower_new path_follower_new;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.init();
        parser parser = new parser("b19dota");
        dataStorage.init(robot.drive, telemetry, this);
        module_master_new.init(hardwareMap);
        path_follower_new = new path_follower_new(robot.drivetrain);
        robot.drive.setPoseEstimate(new Pose2d(39.9, 64.93, Math.PI));

        curve[] curves;
        try {
            curves = parser.getCurves();
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }
        module_master_new.differential.pitchUp();
        module_master_new.differential.rollDefault();
        module_master_new.differential.closeClaw();
        module_master_new.differential.update();

        waitForStart();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        path_follower_new.painter.prepare(packet, fieldOverlay);

        for (curve traj : curves) {
            if (traj != null)
                path_follower_new.painter.drawPolyLine(traj.points, "green");
        }

        path_follower_new.dashboard.sendTelemetryPacket(packet);

        /* INIT */

        while (opModeIsActive()) {
            prepareToScoreHighBasket();
            path_follower_new.followTrajectoryBreak(curves[0], -Math.PI * 3 / 4);
            scoreHighBasket();

            module_master_new.schedule(SET_EXTENSION_LIMIT);
            path_follower_new.followTrajectory(curves[1], Math.PI - Math.toRadians(5.1));
            path_follower_new.goToPos(-14.6, 61.7, Math.PI - Math.toRadians(5.3));

            module_master_new.differential.setPitch(52);
            module_master_new.differential.update();
            waitForCondition(() -> module_master_new.commandQueue.isEmpty());
            waitArmExtension();
            takeSample(false); /* dobor */

            module_master_new.schedule(SET_EXTENSION_CLOSED);
            module_master_new.schedule(SET_ROTATION_LIFT);
            module_master_new.schedule(SET_EXTENSION_LIFT);
            module_master_new.differential.pitchHalfDown();

            path_follower_new.followTrajectoryBreak(curves[2], -Math.PI * 3 / 4);
            waitForCondition(() -> module_master_new.commandQueue.isEmpty());
            waitArmExtension();
            scoreHighBasket(); /* score dobor */
            module_master_new.schedule(SET_EXTENSION_YELLOW1);

            path_follower_new.goToPos(48.3, 49.5, -Math.PI / 2 - Math.toRadians(1.2)); /* sample 1 */
            waitForCondition(() -> module_master_new.commandQueue.isEmpty());
            waitArmExtension();
            module_master_new.differential.setPitch(52);
            module_master_new.differential.update();
            takeSample(false);

            module_master_new.differential.pitchHalfDown();
            module_master_new.schedule(SET_EXTENSION_CLOSED);
            module_master_new.schedule(SET_ROTATION_LIFT);
            module_master_new.schedule(SET_EXTENSION_LIFT);
            path_follower_new.goToPosUnsafe(52.4, 52.4, -Math.PI * 3 / 4);
            waitForCondition(() -> module_master_new.commandQueue.isEmpty());
            waitArmExtension();
            scoreHighBasket();
            module_master_new.schedule(SET_EXTENSION_YELLOW2);

            path_follower_new.goToPos(58, 51.4, -Math.PI / 2); /* sample 2 */
            waitForCondition(() -> module_master_new.commandQueue.isEmpty());
            waitArmExtension();
            module_master_new.differential.setPitch(52);
            module_master_new.differential.update();
            takeSample(false);

            module_master_new.differential.pitchHalfDown();
            module_master_new.schedule(SET_EXTENSION_CLOSED);
            module_master_new.schedule(SET_ROTATION_LIFT);
            module_master_new.schedule(SET_EXTENSION_LIFT);
            path_follower_new.goToPosUnsafe(51.6, 51.6, -Math.PI * 3 / 4);
            waitForCondition(() -> module_master_new.commandQueue.isEmpty());
            waitArmExtension();
            scoreHighBasket();
            module_master_new.schedule(SET_EXTENSION_YELLOW3);

            path_follower_new.goToPos(59.5, 44.3, -Math.PI / 2 + Math.toRadians(21.4)); /* sample 3 */
            waitForCondition(() -> module_master_new.commandQueue.isEmpty());
            waitArmExtension();
            module_master_new.differential.setPitch(52);
            module_master_new.differential.update();
            takeSample(true);

            module_master_new.differential.pitchHalfDown();
            module_master_new.schedule(SET_EXTENSION_CLOSED);
            module_master_new.schedule(SET_ROTATION_LIFT);
            module_master_new.schedule(SET_EXTENSION_LIFT);
            path_follower_new.goToPosUnsafe(51.6, 51.6, -Math.PI * 3 / 4);
            waitForCondition(() -> module_master_new.commandQueue.isEmpty());
            waitArmExtension();
            scoreHighBasket();

            path_follower_new.followTrajectoryBreak(curves[3], -Math.PI, new double[]{0.2}, new int[]{PITCH_FRONT});
            module_master_new.differential.pitchForward();
            module_master_new.differential.setRoll(-8);
            module_master_new.differential.update();
            module_master_new.schedule(SET_EXTENSION_CHAMBER);
            module_master_new.schedule(SET_ROTATION_CHAMBER);
            //waitArmRotation();

            robot.stop();
            module_master_new.stop(dataStorage.telemetry);

            packet = new TelemetryPacket();
            fieldOverlay = packet.fieldOverlay();
            path_follower_new.painter.prepare(packet, fieldOverlay);

            for (curve traj : curves) {
                if (traj != null)
                    path_follower_new.painter.drawPolyLine(traj.points, "green");

                path_follower_new.painter.drawPolyLine(dataStorage.poseHistory.toArray(new Vector2d[0]), "blue");
            }

            path_follower_new.dashboard.sendTelemetryPacket(packet);

            //timer.reset();
            //while(timer.seconds() < 2 && opModeIsActive());
            break;
        }
        robot.stop();
        module_master_new.stop(dataStorage.telemetry);
    }

    private void waitForCondition(BooleanSupplier condition) {
        timer.reset();
        while (!condition.getAsBoolean() && opModeIsActive() && timer.milliseconds() < 5000) {
            module_master_new.update(dataStorage.telemetry);
        }
    }

    private void delay(long milliseconds) {
        timer.reset();
        while (timer.milliseconds() < milliseconds && opModeIsActive()) {
            module_master_new.update(dataStorage.telemetry);
        }
    }

    private void waitArmExtension(){
        waitForCondition(() -> module_master_new.arm.extensionReached());
    }

    private void waitArmRotation(){
        waitForCondition(() -> module_master_new.arm.rotationReached());
    }

    /** PITCH DOWN, CLOSE CLAW, PITCH FORWARD **/
    private void takeSample(boolean third){
        delay(200);
        if (!third) {
            module_master_new.differential.setPitch(52);
            module_master_new.differential.update();
        }
        else {
            module_master_new.differential.setPitch(35);
            module_master_new.differential.setRoll(23);
            module_master_new.differential.update();
            delay(200);
        }

        module_master_new.differential.closeClaw();
        delay(200);
    }

    /** SET DIFFY PITCH DOWN AND ARM ROTATED TO LIFT **/
    private void prepareToScoreHighBasket(){
        module_master_new.differential.pitchDown();
        module_master_new.schedule(SET_ROTATION_LIFT);
        module_master_new.schedule(SET_EXTENSION_LIFT);
    }

    /** SCORE SAMPLE, FOLD EXTENSION AND ROTATION **/
    private void scoreHighBasket(){
        waitArmRotation();
        waitArmExtension();

        /* SCORE SAMPLE */
        module_master_new.differential.pitchScoringBasket();
        delay(300);

        //waitArmRotation();
        module_master_new.differential.openClaw();
        delay(300);

        module_master_new.differential.pitchHalfDown();
        delay(100);

        /* FOLD */
        module_master_new.schedule(SET_EXTENSION_CLOSED);
        module_master_new.schedule(SET_ROTATION_FRONT);
    }
}