package org.firstinspires.ftc.teamcode.opModes.auton;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_LIFT;
import org.firstinspires.ftc.teamcode.data.transfer;

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
public class su4ka extends LinearOpMode {

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
            /* score preload */
            prepareToScore();
            path_follower.followTrajectoryBreak(curves[0], -Math.PI * 3 / 4);
            waitArmRotation();
            scoreHighBasket();

            /* take 1st */
            path_follower.goToPos(52.7, 52.7, -Math.PI / 2 - Math.toRadians(11.5));
            waitArmRotation();
            setExtensionAndWait(arm.extension.YELLOW_1);
            takeSample();
            setExtensionAndWait(arm.extension.CLOSED);

            /* score 1st */
            prepareToScore();
            path_follower.goToPos(52.7, 52.7, -Math.PI * 3 / 4);
            waitArmRotation();
            scoreHighBasket();

            /* take 2nd */
            path_follower.goToPos(58.8, 51.4, -Math.PI / 2);
            waitArmRotation();
            setExtensionAndWait(arm.extension.YELLOW_1);
            takeSample();
            setExtensionAndWait(arm.extension.CLOSED);

            /* score 2nd */
            prepareToScore();
            path_follower.goToPos(52.7, 52.7, -Math.PI * 3 / 4);
            waitArmRotation();
            scoreHighBasket();

            /* take 3rd */
            path_follower.goToPos(59, 47, -Math.PI / 2 + Math.toRadians(19.5));
            waitArmRotation();
            module_master.differential.rollHalfRight();
            setExtensionAndWait(arm.extension.YELLOW_2);
            takeSample();
            setExtensionAndWait(arm.extension.CLOSED);

            /* score 3rd */
            prepareToScore();
            path_follower.goToPos(52.7, 52.7, -Math.PI * 3 / 4);
            waitArmRotation();
            scoreHighBasket();

            /* safety */
            waitArmRotation();
            setExtensionAndWait(arm.extension.CLOSED);

            robot.stop();
            module_master.stop(dataStorage.telemetry);
        }
    }

    private void prepareToScore() {
        module_master.arm.setRotation(arm.rotation.LIFT);
        module_master.differential.pitchDown();
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
        delay(300);
        module_master.differential.pitchDown();
        delay(200);

        module_master.differential.closeClaw();
        delay(200);

        module_master.differential.pitchForward();
    }

    private void setExtensionAndWait(arm.extension extension){
        module_master.arm.setExtension(extension);
        waitArmExtension();
    }
    
    private void scoreHighBasket(){
        setExtensionAndWait(arm.extension.EXTENDED);
        module_master.arm.setRotation(arm.rotation.BACK_HANG1);
        module_master.differential.pitchScoringBasket();
        delay(500);

        module_master.differential.openClaw();
        delay(500);
        module_master.arm.setRotation(arm.rotation.LIFT);

        module_master.differential.pitchHalfDown();
        delay(300);

        setExtensionAndWait(arm.extension.CLOSED);

        module_master.arm.setRotation(arm.rotation.FRONT);
        module_master.differential.pitchForward();
    }
}