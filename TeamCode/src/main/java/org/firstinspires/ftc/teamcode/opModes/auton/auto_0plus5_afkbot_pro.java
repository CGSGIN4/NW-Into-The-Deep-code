package org.firstinspires.ftc.teamcode.opModes.auton;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LOW_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_LIFT;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
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
@Config
public class auto_0plus5_afkbot_pro extends LinearOpMode {
    Robot robot;
    path_follower path_follower;
    ElapsedTime timer = new ElapsedTime();
    Pose2d poseToHold = new Pose2d();
    public static Pose2d scoring_preload = new Pose2d(55.356, 50.35, -Math.PI * 3 / 4);
    public static Pose2d scoring_afk = new Pose2d(55.4, 51.4, -Math.PI * 3 / 4);
    public static Pose2d scoring_1 = new Pose2d(53.6, 53.6, -Math.PI * 3 / 4);
    public static Pose2d scoring_2 = new Pose2d(52, 52, -Math.PI * 3 / 4 + Math.toRadians(6));
    public static Pose2d scoring_3 = new Pose2d(53.2, 49.2, -Math.PI * 3 / 4 - Math.toRadians(4));
    public static Pose2d afk = new Pose2d(29.2, 61.7, Math.PI - Math.toRadians(7.3));
    public static Pose2d sample1 = new Pose2d(47.7, 33.8, -Math.PI / 2);
    public static Pose2d sample2 = new Pose2d(56.85, 33.4, -Math.PI / 2);
    public static Pose2d sample3 = new Pose2d(58.5, 31.4, -Math.PI / 2 + Math.toRadians(42.3));

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
        module_master.differential.pitchDown();
        module_master.differential.rollDefault();
        module_master.differential.closeClawSilno();
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
            poseToHold = path_follower.goToPosWithArmToBasket(scoring_preload.getX(), scoring_preload.getY(), scoring_preload.getHeading());
            //logger.writeLn("----------STARTING SCORING PRELOAD------------");
            scoreHighBasket(afk);

            module_master.differential.openClaw();
            path_follower.goToPos(afk.getX(), afk.getY(), afk.getHeading());
            waitArmRotation();
            waitArmExtension();
            module_master.differential.closeClawSilno();
            delay(200);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);

            module_master.differential.pitchHalfDown();
            module_master.arm.setExtension(arm.extension.EXTENDED);
            poseToHold = path_follower.goToPosWithArmToBasket(scoring_afk.getX(), scoring_afk.getY(), scoring_afk.getHeading());
            scoreHighBasket(sample1); /* score dobor */

            path_follower.goToPosWithArm(sample1.getX(), sample1.getY(), sample1.getHeading()); /* sample 1 */
            //path_follower.goToPosUnsafe(51.4, 51.4, -Math.PI / 2 - Math.toRadians(11.5)); /* sample 1 */
            waitArmRotation();
            waitArmExtension();
            takeSample(false);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            module_master.arm.setExtension(arm.extension.EXTENDED);
            poseToHold = path_follower.goToPosWithArmToBasket(scoring_1.getX(), scoring_1.getY(), scoring_1.getHeading());
            //logger.writeLn("----------STARTING SCORING FIRST------------");
            scoreHighBasket(sample2);

            path_follower.goToPosWithArm(sample2.getX(), sample2.getY(), sample2.getHeading()); /* sample 2 */
            waitArmRotation();
            waitArmExtension();
            takeSample(false);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            module_master.arm.setExtension(arm.extension.EXTENDED);
            poseToHold = path_follower.goToPosWithArmToBasket(scoring_2.getX(), scoring_2.getY(), scoring_2.getHeading());
            //logger.writeLn("----------STARTING SCORING SECOND------------");
            scoreHighBasket(sample3);
            module_master.differential.setPitch(6);
            module_master.differential.setRoll(35);
            module_master.differential.update();
            module_master.arm.setExtension(arm.extension.SUPPORT);

            module_master.arm.ROTATION_PIDF.p /= 1.35;

            path_follower.goToPosWithArmThirdSample(sample3.getX(), sample3.getY(), sample3.getHeading()); /* intaking yellow 3 */
            module_master.arm.setExtension(arm.extension.YELLOW_3_PRO);
            //module_master.arm.setExtension(arm.extension.YELLOW_3_PRO);
            waitArmRotation();
            waitArmExtension();
            takeSample(true);
            module_master.differential.rollDefault();

            module_master.arm.ROTATION_PIDF.p *= 1.35;

            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            path_follower.velocity_calculator.setThirdSampleRotationCoeffs();
            poseToHold = path_follower.goToPosWithArmToBasket(scoring_3.getX(), scoring_3.getY(), scoring_3.getHeading());
            path_follower.velocity_calculator.setDefaultRotationCoeffs();
            //logger.writeLn("----------STARTING SCORING THIRD------------");
            scoreHighBasket(new Pose2d(24, 0, scoring_3.getHeading()));

            path_follower.p_trans_coef *= 2;

            /* go to dobor1 */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 42, -Math.PI, new double[]{0.2, 0.2, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            path_follower.goToPosVeryUnsafe(22.2, 7, -Math.PI);

            module_master.differential.pitchDown();
            module_master.differential.rollDefault();
            delay(200);
            takeSample(true);
            module_master.differential.rollDefault();
            module_master.differential.setPitch(135);
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor1 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.1, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(51.9, 51.8, -Math.PI * 3 / 4);
            scoreHighBasket(new Pose2d(25, 0, -Math.PI * 3 / 4));

            /* go to dobor2 */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 42, -Math.PI, new double[]{0.2, 0.2, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            path_follower.goToPosVeryUnsafe(22.2, 9, -Math.PI);

            module_master.differential.pitchDown();
            module_master.differential.rollDefault();
            delay(200);
            takeSample(true);
            module_master.differential.rollDefault();
            module_master.differential.setPitch(135);
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor2 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.1, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(51.2, 49.8, -Math.PI * 3 / 4);
            scoreHighBasket(new Pose2d(28, 0, -Math.PI * 3 / 4));

            /* go to dobor3 */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 42, -Math.PI, new double[]{0.2, 0.2, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            path_follower.goToPosVeryUnsafe(22.2, 7.5, -Math.PI + Math.toRadians(5));

            module_master.differential.pitchDown();
            module_master.differential.rollDefault();
            delay(200);
            takeSample(true);
            module_master.differential.rollDefault();
            module_master.differential.setPitch(135);
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor3 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.1, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(50.2, 50.8, -Math.PI * 3 / 4 - Math.toRadians(11));
            scoreHighBasket(new Pose2d(33, 0, -Math.PI * 3 / 4 - Math.toRadians(11)));

            /* go to dobor4 */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 50, -Math.PI, new double[]{0.2, 0.2, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            path_follower.goToPosVeryUnsafe(22.2, 7.5, -Math.PI + Math.toRadians(5));

            module_master.differential.pitchDown();
            module_master.differential.rollDefault();
            delay(200);
            takeSample(true);
            module_master.differential.rollDefault();
            module_master.differential.setPitch(135);
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor4 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.1, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(50.2, 50.8, -Math.PI * 3 / 4 - Math.toRadians(11));
            scoreHighBasket(new Pose2d(37, 0, -Math.PI * 3 / 4 - Math.toRadians(7)));

            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 42, -Math.PI, new double[]{0.2, 0.2, 0.8, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER, SET_ROTATION_CHAMBER});
            path_follower.goToPosVeryUnsafe(20, 6, -Math.PI);

            module_master.arm.setRotation(arm.rotation.CHAMBER);

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
            if (poseToHold.getX() != -100)
                path_follower.holdPosAsync(poseToHold);
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
        delay(100);

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
    private void scoreHighBasket() {
        waitArmRotation();
        waitArmExtension();

        /* SCORE SAMPLE */
        module_master.differential.setPitch(180);
        module_master.differential.update();
        delay(150);

        //waitArmRotation();
        module_master.differential.openClaw();
        delay(100);

        module_master.differential.pitchHalfDown();
        //delay(100);

        /* FOLD */
        module_master.arm.setExtension(arm.extension.CLOSED);
        module_master.arm.setRotation(arm.rotation.FRONT);
        poseToHold = new Pose2d(-100, -100, 10);
    }

    private void scoreHighBasket(Pose2d next) {
        waitArmRotation();
        waitArmExtension();

        /* SCORE SAMPLE */
        module_master.differential.pitchScoringBasket();
        delay(100);
        poseToHold = next;
        delay(50);

        //waitArmRotation();
        module_master.differential.openClaw();
        poseToHold = next;
        delay(100);

        module_master.differential.pitchHalfDown();
        //delay(100);

        /* FOLD */
        module_master.arm.setExtension(arm.extension.CLOSED);
        module_master.arm.setRotation(arm.rotation.FRONT);
        poseToHold = new Pose2d(-100, -100, 10);
    }
}