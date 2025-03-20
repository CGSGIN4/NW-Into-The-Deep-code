package org.firstinspires.ftc.teamcode.opModes.auton;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_UP;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_CHAMBER;
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
import org.firstinspires.ftc.teamcode.subsystems.vision.LLSmotritel;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.IOException;
import java.util.function.BooleanSupplier;

@Autonomous
@Config
public class afkbot_pro_red extends LinearOpMode {
    Robot robot;
    path_follower path_follower;
    ElapsedTime timer = new ElapsedTime();
    Pose2d poseToHold = new Pose2d();
    public static Pose2d scoring_preload = new Pose2d(54.356, 48.35, -Math.PI * 3 / 4);
    public static Pose2d scoring_afk = new Pose2d(54.9, 49.61, -Math.PI * 3 / 4);
    public static Pose2d scoring_1 = new Pose2d(53, 50.86, -Math.PI * 3 / 4);
    public static Pose2d scoring_2 = new Pose2d(50, 51.8, -Math.PI * 3 / 4);
    public static Pose2d scoring_3 = new Pose2d(52.2, 47.7, -Math.PI * 3 / 4 - Math.toRadians(4.7));
    public static Pose2d afk = new Pose2d(33.3, 61.1, Math.PI - Math.toRadians(7.3));
    public static Pose2d sample1 = new Pose2d(47.3, 34.1, -Math.PI / 2);
    public static Pose2d sample2 = new Pose2d(56.45, 33.4, -Math.PI / 2);
    public static Pose2d sample3 = new Pose2d(58.5, 31.4, -Math.PI / 2 + Math.toRadians(42.3));

    @Override
    public void runOpMode() throws InterruptedException {
        LLSmotritel smotritel = new LLSmotritel(hardwareMap, 0);
        smotritel.stopStreaming();
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
            module_master.differential.setPitch(25);
            module_master.differential.setRoll(-25);
            module_master.differential.update();
            module_master.arm.setExtension(arm.extension.SUPPORT);

            module_master.arm.ROTATION_PIDF.p /= 1.35;

            path_follower.goToPosWithArmThirdSample(afk.getX(), afk.getY(), afk.getHeading()); /* intaking afk */
            module_master.arm.setExtension(arm.extension.YELLOW_3_PRO);
            //module_master.arm.setExtension(arm.extension.YELLOW_3_PRO);
            waitArmRotation();
            waitArmExtension();
            module_master.arm.ROTATION_PIDF.p *= 1.35;
            module_master.differential.closeClawSilno();
            delay(200);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);

            module_master.differential.pitchHalfDown();
            module_master.differential.rollDefault();
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

            /* go to dobor1 */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 39,-Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
            smotritel.startStreaming();
            path_follower.velocity_calculator.P_ROTATION_COEF /= 1.5;
            path_follower.goToPosVeryUnsafe(16, -2, -Math.PI);
            if (!takeFromSubmersible(smotritel, 1))
                instapark();
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor1 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.2, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(56.7, 44.26, -Math.PI * 3 / 4);
            waitArmExtension();
            scoreHighBasket(new Pose2d(32, 0, -Math.PI * 3 / 4));

            /* go to dobor2 */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 70,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
            smotritel.startStreaming();
            path_follower.velocity_calculator.P_ROTATION_COEF /= 1.5;
            path_follower.goToPosVeryUnsafe(17.3, -3, -Math.PI);
            if (!takeFromSubmersible(smotritel, 2))
                instapark();
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor2 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.2, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(56.2, 43.8, -Math.PI * 3 / 4 - Math.toRadians(4));
            waitArmExtension();
            scoreHighBasket(new Pose2d(36, 0, -Math.PI * 3 / 4 - Math.toRadians(4)));

            /* go to dobor3 */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 84,39, -Math.PI, new double[]{0.3, 0.3}, new int[]{SET_ROTATION_CHAMBER, PITCH_UP});
            smotritel.startStreaming();
            path_follower.velocity_calculator.P_ROTATION_COEF /= 1.5;
            path_follower.goToPosVeryUnsafe(18, -2, -Math.PI);
            if (!takeFromSubmersible(smotritel, 2))
                instapark();
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            delay(300);

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

        delay(150);
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
        delay(155);
        poseToHold = next;
        delay(20);

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

    private boolean takeFromSubmersible(LLSmotritel smotritel, int call){
        boolean smestilsya = false;
        boolean nevidel = false;
        smotritel.startStreaming();
        robot.drivetrain.applyVectorFieldCentric(new Vector2d(-1, 0).rotated(Math.toRadians(90)), 0);
        delay(300);
        smotritel.startSnapshot();
        Pose2d offsets = smotritel.getSampleOffsets(0);
        dataStorage.updateData();
        Pose2d snapshotPos = new Pose2d(dataStorage.RobotPose, dataStorage.RobotWorldHeading);

        if ((offsets.getX() == -100 || Math.abs(LLSmotritel.getTicks(offsets)) > module_master.arm.EXTENSION_FRONT_MAX) && opModeIsActive())
        {
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            path_follower.goToPos(snapshotPos.getX() + 0.1, dataStorage.RobotWorldY - 4, -Math.PI);
            smotritel.startSnapshot();
            offsets = smotritel.getSampleOffsets(0);
            nevidel = true;
        }
        if (nevidel) {
            if (offsets.getX() == -100)
                return false;
            robot.drivetrain.applyVectorFieldCentric(new Vector2d(-1, 0).rotated(Math.toRadians(90)), 0);
            delay(300);
        }
        Pose2d rememberedOffsets = offsets;
        offsets = smotritel.getSampleOffsets(0);
        smotritel.startSnapshot();
        dataStorage.telemetry.addData("angle", dataStorage.RobotWorldHeading);

        while (Math.abs(offsets.getX()) > 15 && opModeIsActive()) {
            if (offsets.getX() == -100) {
                offsets = rememberedOffsets;
                break;
            }
            dataStorage.updateData();
            path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets), -Math.PI);
            offsets = smotritel.getSampleOffsets(0);
            smotritel.startSnapshot();
            smestilsya = true;
        }
        if (smestilsya) {
            delay(300);
            offsets = smotritel.getSampleOffsets(0);
            smotritel.startSnapshot();
        }

        module_master.differential.setPitch(0);
        module_master.differential.setRoll(offsets.getHeading());
        module_master.differential.update();

        module_master.arm.setRotation(arm.rotation.FRONT);
        module_master.arm.pidExtend(LLSmotritel.getTicks(offsets));
        dataStorage.drive.setPoseEstimate(new Pose2d(22, dataStorage.RobotWorldY, dataStorage.RobotWorldHeading));
        dataStorage.updateData();
        if (dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets) < -8)
            return false;
        path_follower.goToPosPrecise(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets), -Math.PI);
        path_follower.velocity_calculator.P_ROTATION_COEF *= 1.5;

        while (Math.abs(module_master.arm.extensionMotor.getCurrentPosition() - LLSmotritel.getTicks(offsets)) + module_master.arm.offset > 13 && opModeIsActive()) {
            if (module_master.arm.rotationReached())
                module_master.arm.pidExtend(LLSmotritel.getTicks(offsets));
            module_master.update(dataStorage.telemetry);
        }

        /*
        module_master.differential.setPitch(0);
        module_master.differential.update();
        timer.reset();
        while (timer.milliseconds() < 200) module_master.update(dataStorage.telemetry);
        module_master.differential.setRoll(offsets.getHeading());
        module_master.differential.update();
        timer.reset();
        while (timer.milliseconds() < 200) module_master.update(dataStorage.telemetry);
        */
        waitArmRotation();
        waitArmExtension();
        timer.reset();
        delay(25);
        module_master.differential.closeClaw();
        timer.reset();
        delay(150);
        module_master.differential.setPitch(135);
        module_master.differential.setRoll(-11);
        module_master.differential.update();

        smotritel.stopStreaming();
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
}