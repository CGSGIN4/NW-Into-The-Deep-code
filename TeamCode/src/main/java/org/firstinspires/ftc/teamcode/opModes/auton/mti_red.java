package org.firstinspires.ftc.teamcode.opModes.auton;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLOSE_CLAW_VERY_SILNO;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_SCORING_BASKET;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_UP;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_FRONT;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.data.transfer;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.vision.LLSmotritel;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.IOException;
import java.util.Vector;
import java.util.function.BooleanSupplier;

@Autonomous
public class mti_red extends LinearOpMode {
    boolean willGoToLast = true;

    Robot robot;
    path_follower path_follower;
    ElapsedTime timer = new ElapsedTime();
    Pose2d poseToHold = new Pose2d();
    Pose2d sample1 = new Pose2d(48.1, 35.25, -Math.PI / 2);
    Pose2d sample2 = new Pose2d(57.8, 34.65, -Math.PI / 2);
    Pose2d sample3 = new Pose2d(59.7, 32.5, -Math.PI / 2 + Math.toRadians(42.3));
    Pose2d nextPos = new Pose2d(-100, 100, 0);

    //SampleDetectionProcessor sampleDetection = new SampleDetectionProcessor();

    @Override
    public void runOpMode() throws InterruptedException {
        //logger.init();
        LLSmotritel smotritel = new LLSmotritel(hardwareMap, 0);
        //smotritel.stopStreaming();
        robot = new Robot(hardwareMap);
        robot.init();
        parser parser = new parser("b19dotaMTI");
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
            poseToHold = path_follower.goToPosWithArmToBasketSu4kaMTI(54.856, 50.85, -Math.PI * 3 / 4);
            //logger.writeLn("----------STARTING SCORING PRELOAD------------");
            scoreHighBasketPreload(sample1);

            module_master.differential.openClaw();
            path_follower.velocity_calculator.p_trans_coef /= 1.5;
            path_follower.goToPosWithArmMTI(sample1.getX(), sample1.getY(), sample1.getHeading()); /* sample 1 */
            path_follower.velocity_calculator.p_trans_coef *= 1.5;
            //path_follower.goToPosUnsafe(51.4, 51.4, -Math.PI / 2 - Math.toRadians(11.5)); /* sample 1 */
            while (module_master.arm.rotationState != arm.rotation.RESET && opModeIsActive())
                module_master.arm.update();
            waitArmExtension();
            takeSample(false);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            //module_master.arm.setExtension(arm.extension.EXTENDED);
            poseToHold = path_follower.goToPosWithArmToBasketSu4ka12MTI(52.9, 52.9, -Math.PI * 3 / 4 + Math.toRadians(3));
            //logger.writeLn("----------STARTING SCORING FIRST------------");
            scoreHighBasket(sample2);

            path_follower.goToPosWithArmMTI(sample2.getX(), sample2.getY(), sample2.getHeading()); /* sample 2 */
            waitArmRotation();
            waitArmExtension();
            takeSample(false);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            //module_master.arm.setExtension(arm.extension.EXTENDED);
            poseToHold = path_follower.goToPosWithArmToBasketSu4ka12MTI(54.4, 50.4, -Math.PI * 3 / 4 + Math.toRadians(12));
            //logger.writeLn("----------STARTING SCORING SECOND------------");
            scoreHighBasket(sample3);
            module_master.differential.setPitch(6);
            module_master.differential.setRoll(35);
            module_master.differential.update();
            module_master.arm.setExtension(arm.extension.SUPPORT);

            module_master.arm.ROTATION_PIDF.p /= 1.75;

            path_follower.velocity_calculator.d_trans_coef *= 1.3;
            path_follower.goToPosWithArmThirdSample(sample3.getX(), sample3.getY(), sample3.getHeading()); /* intaking yellow 3 */
            path_follower.velocity_calculator.d_trans_coef /= 1.3;
            module_master.arm.setExtension(arm.extension.YELLOW_3_PRO);
            //module_master.arm.setExtension(arm.extension.YELLOW_3_PRO);
            waitArmRotation();
            waitArmExtension();
            takeSample(true);
            module_master.differential.rollDefault();

            module_master.arm.ROTATION_PIDF.p *= 1.45;

            //module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            path_follower.velocity_calculator.setThirdSampleRotationCoeffs();
            //module_master.arm.setExtension(arm.extension.EXTENDED);
            poseToHold = path_follower.goToPosWithArmToBasketSu4kaThirdSampleMTI(52.6, 48.6, -Math.PI * 3 / 4);
            path_follower.velocity_calculator.setDefaultRotationCoeffs();
            //logger.writeLn("----------STARTING SCORING THIRD------------");
            scoreHighBasket(new Pose2d(28, 0, -Math.PI * 3 / 4 - Math.toRadians(2)));

            /* go to dobor1 */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 39,-Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
            smotritel.startStreaming();
            if (!takeFromSubmersible(smotritel, 1))
                instapark();
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor1 */
            module_master.arm.setRotation(arm.rotation.LIFT);
            path_follower.followTrajectoryBackwardsPercentage(curves[5], 85, new double[]{0.09, 0.15, 0.45, 0.82}, new int[]{SET_EXTENSION_LIFT, PITCH_DOWN, CLOSE_CLAW_VERY_SILNO, PITCH_FRONT});
            path_follower.velocity_calculator.d_trans_coef *= 1.2;
            /* CHANGE ME PLEASE */
            path_follower.goToPosWithArmToBasketSu4kaDoborMTI(53, 40, -Math.PI * 3 / 4 + Math.toRadians(10));
            /* CHANGE ME PLEASE */
            path_follower.velocity_calculator.d_trans_coef /= 1.2;
            waitArmExtension();
            scoreHighBasketFast(new Pose2d(36, 0, -Math.PI * 3 / 4 + Math.toRadians(10)));

            /* go to dobor2 */
            if (nextPos.getX() != -100) {
                try {
                    curve c = new curve(new Vector2d[]{curves[6].nodes[0], curves[6].nodes[1], curves[6].nodes[2], nextPos.vec()});
                    path_follower.followTrajectoryForwardsPercentageHypeAngleControl(c, 90,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
                } catch (Exception ignored) {
                    path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[6], 90,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
                }
            }
            else
                path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[6], 90,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
            smotritel.startStreaming();
            //path_follower.goToPosVeryUnsafe(18.3, 5.5, -Math.PI + Math.toRadians(2));
            if (!takeFromSubmersible(smotritel, 2))
                instapark();
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor2 */
            module_master.arm.setRotation(arm.rotation.LIFT);
            path_follower.followTrajectoryBackwardsPercentage(curves[5], 85, new double[]{0.09, 0.15, 0.45, 0.82}, new int[]{SET_EXTENSION_LIFT, PITCH_DOWN, CLOSE_CLAW_VERY_SILNO, PITCH_FRONT});
            path_follower.velocity_calculator.d_trans_coef *= 1.2;
            /* CHANGE ME PLEASE */
            path_follower.goToPosWithArmToBasketSu4kaDoborMTI(47.7, 41.4, -Math.PI * 3 / 4 + Math.toRadians(5));
            /* CHANGE ME PLEASE */
            path_follower.velocity_calculator.d_trans_coef /= 1.2;
            waitArmExtension();
            scoreHighBasketFast(new Pose2d(36, 0, -Math.PI * 3 / 4 + Math.toRadians(5)));

            /* go to dobor3 */
            if (nextPos.getX() != -100) {
                try {
                    curve c = new curve(new Vector2d[]{curves[4].nodes[0], curves[4].nodes[1], curves[4].nodes[2], nextPos.vec()});
                    path_follower.followTrajectoryForwardsPercentageHypeAngleControl(c, 90,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
                } catch (Exception ignored) {
                    path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
                }
            }
            else
                path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
            smotritel.startStreaming();
            if (!takeFromSubmersible(smotritel, 3))
                instapark();
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor3 */
            module_master.arm.setRotation(arm.rotation.LIFT);
            path_follower.followTrajectoryBackwardsPercentage(curves[5], 65, new double[]{0.09, 0.15, 0.45, 0.62}, new int[]{SET_EXTENSION_LIFT, PITCH_DOWN, CLOSE_CLAW_VERY_SILNO, PITCH_FRONT});
            path_follower.velocity_calculator.d_trans_coef *= 1.2;
            /* CHANGE ME PLEASE */
            path_follower.goToPosWithArmToBasketSu4kaDoborMTI(46.7, 42.3, -Math.PI * 3 / 4 + Math.toRadians(10));
            /* CHANGE ME PLEASE */
            path_follower.velocity_calculator.d_trans_coef /= 1.2;
            waitArmExtension();
            scoreHighBasketFast(new Pose2d(36, 0, -Math.PI * 3 / 4));

            /* go to dobor4 */
            if (nextPos.getX() != -100) {
                try {
                    curve c = new curve(new Vector2d[]{curves[7].nodes[0], curves[7].nodes[1], curves[7].nodes[2], nextPos.vec()});
                    path_follower.followTrajectoryForwardsPercentageHypeAngleControl(c, 90,26, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
                } catch (Exception ignored) {
                    path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[7], 90,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
                }
            }
            else
                path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[7], 90,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
            smotritel.startStreaming();
            if (!takeFromSubmersible(smotritel, 4))
                instapark();
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor4 */
            module_master.arm.setRotation(arm.rotation.LIFT);
            path_follower.followTrajectoryBackwardsPercentage(curves[5], 65, new double[]{0.02, 0.15, 0.35, 0.52}, new int[]{SET_EXTENSION_LIFT, PITCH_DOWN, CLOSE_CLAW_VERY_SILNO, PITCH_FRONT});
            module_master.differential.pitchScoringBasketDobor();
            path_follower.velocity_calculator.d_trans_coef *= 1.2;
            /* CHANGE ME PLEASE */
            path_follower.goToPosWithArmToBasketSu4kaDoborMTI(53.7, 43.3, -Math.PI * 3 / 4 + Math.toRadians(5));
            /* CHANGE ME PLEASE */
            path_follower.velocity_calculator.d_trans_coef /= 1.2;
            waitArmExtension();
            scoreHighBasketFast(new Pose2d(36, 0, -Math.PI * 3 / 4));

            /* go to park */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 42, -Math.PI, new double[]{0.2, 0.2, 0.8, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER, SET_ROTATION_CHAMBER});
            path_follower.goToPosVeryUnsafe(20, 3, -Math.PI);
            module_master.arm.setRotation(arm.rotation.CHAMBER);
            while (opModeIsActive()) {
                module_master.arm.update();
                module_master.arm.manuallyExtend(0);
                robot.stop();
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
        //smotritel.stopDashboardStream();
        /*smotritel.stopStream();*/
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
    private void scoreHighBasketFast(Pose2d next) {
        waitArmRotation();
        waitArmExtension();

        /* SCORE SAMPLE */
        module_master.differential.setPitch(137);
        module_master.differential.update();
        delay(0);
        poseToHold = next;
        delay(10);

        //waitArmRotation();
        module_master.differential.openClaw();
        poseToHold = next;
        delay(85);

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
        delay(50);
        poseToHold = next;
        delay(20);

        //waitArmRotation();
        module_master.differential.openClaw();
        poseToHold = next;
        delay(125);

        module_master.differential.pitchHalfDown();

        /* FOLD */
        module_master.arm.setExtension(arm.extension.CLOSED);
        module_master.arm.setRotation(arm.rotation.FRONT);
        poseToHold = new Pose2d(-100, -100, 10);
    }

    private void scoreHighBasketPreload(Pose2d next) {
        waitArmRotation();
        waitArmExtension();

        /* SCORE SAMPLE */
        module_master.differential.pitchScoringBasket();
        delay(50);
        poseToHold = next;
        delay(5);

        //waitArmRotation();
        module_master.differential.openClaw();
        poseToHold = next;
        delay(125);

        module_master.differential.pitchHalfDown();
        //delay(100);

        /* FOLD */
        module_master.arm.setExtension(arm.extension.CLOSED);
        module_master.arm.setRotation(arm.rotation.FRONT);
        poseToHold = new Pose2d(-100, -100, 10);
    }

    private boolean takeFromSubmersible(LLSmotritel smotritel, int call){
        nextPos = new Pose2d(-100, 100, 0);
        boolean nevidel = false;
        smotritel.startStreaming();
        if (call == 4)
            robot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.55, 0).rotated(Math.toRadians(90)), 0);
        else
            robot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.25, 0).rotated(Math.toRadians(90)), 0);
        dataStorage.updateData();
        while ((dataStorage.RobotVelocity.norm() > 2 || !headingAroundPi(0.1)) && opModeIsActive()) {
            if (!headingAroundPi(0.1))
                robot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.55, 0).rotated(Math.toRadians(90)), 0);
            delay(1);
        }
        dataStorage.photoVel = dataStorage.RobotVelocity.norm();
        while (!smotritel.startSnapshot() && opModeIsActive())
        {
            delay(1);
            dataStorage.telemetry.addData("alllo", "alllooo");
        }
        Pose2d offsets = smotritel.getSampleOffsets_beta(0, dataStorage.RobotWorldX, dataStorage.RobotWorldY);

        Pose2d secondOffsets = smotritel.getSecondOffsets();
        if (secondOffsets.getX() != -100)
            nextPos = new Pose2d(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(secondOffsets) * 1.2, -Math.PI);

        Pose2d snapshotPos = new Pose2d(dataStorage.RobotPose, dataStorage.RobotWorldHeading);

        if ((offsets.getX() == -100 || Math.abs(LLSmotritel.getTicks(offsets)) > module_master.arm.EXTENSION_FRONT_MAX) && opModeIsActive())
        {
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            path_follower.goToPos(snapshotPos.getX(), dataStorage.RobotWorldY - 7, -Math.PI);
            robot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.45, 0).rotated(Math.toRadians(90)), 0);
            while ((dataStorage.RobotVelocity.norm() > 2 || !headingAroundPi(0.1)) && opModeIsActive()) {
                if (!headingAroundPi(0.1))
                    robot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.55, 0).rotated(Math.toRadians(90)), 0);
                delay(1);
            }
            while (!smotritel.startSnapshot() && opModeIsActive()) delay(1);
            offsets = smotritel.getSampleOffsets_beta(0, dataStorage.RobotWorldX, dataStorage.RobotWorldY);

            secondOffsets = smotritel.getSecondOffsets();
            if (nextPos.getY() < dataStorage.RobotWorldY + smotritel.getTranslationalOffset(secondOffsets) * 1.2 && secondOffsets.getX() != -100)
                nextPos = new Pose2d(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(secondOffsets) * 1.2, -Math.PI);

            nevidel = true;
        }
        if (nevidel) {
            willGoToLast = false;
            if (offsets.getX() == -100)
                return false;
        }

        while (Math.abs(offsets.getX()) > 15 && opModeIsActive()) {
            willGoToLast = false;

            dataStorage.updateData();
            path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets), -Math.PI);
            robot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.45, 0).rotated(Math.toRadians(90)), 0);
            while ((dataStorage.RobotVelocity.norm() > 2 || !headingAroundPi(0.1)) && opModeIsActive()) {
                if (!headingAroundPi(0.1))
                    robot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.55, 0).rotated(Math.toRadians(90)), 0);
                delay(1);
            }
            while (!smotritel.startSnapshot() && opModeIsActive()) delay(1);
            offsets = smotritel.getSampleOffsets_beta(0, dataStorage.RobotWorldX, dataStorage.RobotWorldY);
            secondOffsets = smotritel.getSecondOffsets();
            if (nextPos.getY() < dataStorage.RobotWorldY + smotritel.getTranslationalOffset(secondOffsets) * 1.2)
                nextPos = new Pose2d(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(secondOffsets) * 1.2, -Math.PI);
        }

        module_master.differential.setPitch(0);
        module_master.differential.setRoll(offsets.getHeading());
        module_master.differential.update();

        module_master.arm.setRotation(arm.rotation.FRONT);
        module_master.arm.pidExtend(LLSmotritel.getTicks(offsets));
        //dataStorage.drive.setPoseEstimate(new Pose2d(22, dataStorage.RobotWorldY, dataStorage.RobotWorldHeading));
        dataStorage.updateData();
        if (dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets) * 1.2 < -14)
            return false;
        path_follower.goToPosPreciseMTI(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets) + smotritel.getTranslationalOffset(offsets) * 0.05, -Math.PI);

        while (Math.abs(module_master.arm.extensionMotor.getCurrentPosition() - LLSmotritel.getTicks(offsets)) + module_master.arm.offset > 13 && opModeIsActive()) {
            if (module_master.arm.rotationReached())
                module_master.arm.pidExtend(LLSmotritel.getTicks(offsets));
            module_master.update(dataStorage.telemetry);
        }

        waitArmRotation();
        waitArmExtension();
        timer.reset();
        delay(0);
        module_master.differential.closeClaw();
        timer.reset();
        delay(125);
        module_master.differential.setPitch(70);
        module_master.differential.setRoll(-11);
        module_master.differential.update();

        //smotritel.stopStreaming();
        return true;
    }

    /*
    private void takeVideo(double angle){
        module_master.differential.pitchDown();
        sleep(300);
        module_master.differential.setRoll(differential.geomToDifAngle(angle));
        module_master.differential.update();
        sleep(300);
        module_master.differential.closeClaw();
        sleep(200);
        module_master.differential.rollDefault();
        module_master.differential.pitchForward();
        module_master.differential.update();
    }
    */
    private void instapark(){
        module_master.arm.setRotation(arm.rotation.FRONT);
        waitArmRotation();
        module_master.arm.setExtension(arm.extension.HIGH_CHAMBER);
        module_master.arm.setRotation(arm.rotation.CHAMBER);
        waitArmRotation();
        waitArmExtension();

        while (opModeIsActive()) {
            module_master.arm.update();
            module_master.arm.manuallyExtend(0);
        }
        robot.stop();
        transfer.angle = dataStorage.RobotWorldHeading;
    }

    private boolean headingAroundPi(double threshold)
    {
        return (Math.abs(dataStorage.RobotWorldHeading + Math.PI) < threshold || Math.abs(dataStorage.RobotWorldHeading + Math.PI - 6.28) < threshold);
    }
}