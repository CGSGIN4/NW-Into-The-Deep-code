package org.firstinspires.ftc.teamcode.opModes.auton;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LOW_CHAMBER;
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
import org.firstinspires.ftc.teamcode.data.transfer;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.IOException;
import java.util.function.BooleanSupplier;

@Autonomous
public class auto_0plus5_afkbot_pro extends LinearOpMode {
    Robot robot;
    path_follower path_follower;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //logger.init();
        robot = new Robot(hardwareMap);
        robot.init();
        parser parser = new parser("b19dotaPRO");
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
            path_follower.goToPosWithArmToBasket(53.356, 48.35, -Math.PI * 3 / 4);
            //logger.writeLn("----------STARTING SCORING PRELOAD------------");
            scoreHighBasket();

            module_master.differential.openClaw();
            path_follower.goToPos(30.3, 61.7, Math.PI - Math.toRadians(7.3));
            waitArmRotation();
            waitArmExtension();
            module_master.differential.closeClaw();
            delay(200);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);

            path_follower.goToPosWithArmToBasket(53.4, 49.4, -Math.PI * 3 / 4);
            module_master.differential.pitchHalfDown();
            module_master.arm.setExtension(arm.extension.EXTENDED);
            waitArmExtension();
            scoreHighBasket(); /* score dobor */

            path_follower.goToPosWithArm(47.4, 34.5, -Math.PI / 2); /* sample 1 */
            //path_follower.goToPosUnsafe(51.4, 51.4, -Math.PI / 2 - Math.toRadians(11.5)); /* sample 1 */
            waitArmRotation();
            waitArmExtension();
            takeSample(false);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            path_follower.goToPosWithArmToBasket(51.4, 51.4, -Math.PI * 3 / 4);
            setExtensionAndWait(arm.extension.EXTENDED);
            //logger.writeLn("----------STARTING SCORING FIRST------------");
            scoreHighBasket();

            path_follower.goToPosWithArm(57, 33.2, -Math.PI / 2); /* sample 2 */
            waitArmRotation();
            waitArmExtension();
            takeSample(false);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            path_follower.goToPosWithArmToBasket(51.6, 51.6, -Math.PI * 3 / 4);
            setExtensionAndWait(arm.extension.EXTENDED);
            //logger.writeLn("----------STARTING SCORING SECOND------------");
            scoreHighBasket();
            module_master.differential.setPitch(6);
            module_master.differential.setRoll(35);
            module_master.differential.update();
            module_master.arm.setExtension(arm.extension.SUPPORT);

            path_follower.goToPosWithArmThirdSample(58.7, 32, -Math.PI / 2 + Math.toRadians(40.3)); /* intaking yellow 3 */
            module_master.arm.setExtension(arm.extension.YELLOW_3_PRO);
            //module_master.arm.setExtension(arm.extension.YELLOW_3_PRO);
            waitArmRotation();
            waitArmExtension();
            takeSample(true);
            module_master.differential.rollDefault();

            //module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            path_follower.velocity_calculator.setThirdSampleRotationCoeffs();
            path_follower.goToPosWithArmToBasket(52.2, 47.2, -Math.PI * 3 / 4 + Math.toRadians(10));
            path_follower.velocity_calculator.setDefaultRotationCoeffs();
            setExtensionAndWait(arm.extension.EXTENDED);
            //logger.writeLn("----------STARTING SCORING THIRD------------");
            scoreHighBasket();

            /* go to dobor1 */
            path_follower.followTrajectoryForwards(curves[4], new double[]{0.2, 0.2, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            //module_master.arm.setExtension(arm.extension.HIGH_CHAMBER);
            path_follower.goToPosVeryUnsafe(21, 6, -Math.PI - Math.toRadians(5));

            module_master.differential.pitchDown();
            module_master.differential.rollHalfRight();
            delay(200);
            takeSample(true);
            module_master.differential.rollDefault();
            module_master.differential.setPitch(135);
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor1 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.2, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(50.2, 46.8, -Math.PI * 3 / 4);
            scoreHighBasket();

            /* go to dobor2 */
            path_follower.followTrajectoryForwards(curves[4], 84, new double[]{0.2, 0.2, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            //module_master.arm.setExtension(arm.extension.HIGH_CHAMBER);
            path_follower.goToPosVeryUnsafe(20.5, 6, -Math.PI - Math.toRadians(15));

            module_master.differential.pitchDown();
            module_master.differential.rollHalfLeft();
            delay(200);
            takeSample(true);
            module_master.differential.rollDefault();
            module_master.differential.setPitch(135);
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor2 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.2, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(50.2, 48.8, -Math.PI * 3 / 4);
            scoreHighBasket();

            path_follower.followTrajectoryForwards(curves[4], new double[]{0.2, 0.2, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            path_follower.goToPosVeryUnsafe(20, 6, -Math.PI);

            module_master.arm.setRotation(arm.rotation.CHAMBER);
            //module_master.arm.setRotation(arm.rotation.FRONT);

            while (opModeIsActive()) {
                module_master.arm.update();
                module_master.arm.manuallyExtend(0);
            }
            robot.stop();
            //module_master.stop(dataStorage.telemetry);
            transfer.angle = robot.drive.getPoseEstimate().getHeading();

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
        transfer.armExtensionPos = module_master.arm.extensionMotor.getCurrentPosition();
        //logger.writeLn("offset: " + transfer.armExtensionPos);
        //logger.close();
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
            ;
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
        delay(200);

        module_master.differential.pitchHalfDown();
        delay(100);

        /* FOLD */
        module_master.arm.setExtension(arm.extension.CLOSED);
        module_master.arm.setRotation(arm.rotation.FRONT);
    }
}