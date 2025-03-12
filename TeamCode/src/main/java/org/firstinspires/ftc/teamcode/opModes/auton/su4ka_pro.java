package org.firstinspires.ftc.teamcode.opModes.auton;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CAMERA;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LOW_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_CAMERA;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_LIFT;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.data.transfer;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.vision.Sample;
import org.firstinspires.ftc.teamcode.subsystems.vision.SampleDetectionProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.Smotritel;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.BlackPipeline;
import org.firstinspires.ftc.teamcode.utils.logger;
import org.firstinspires.ftc.teamcode.utils.parser;
import org.firstinspires.ftc.vision.VisionPortal;

import java.io.IOException;
import java.util.function.BooleanSupplier;

@Autonomous
public class su4ka_pro extends LinearOpMode {

    Robot robot;
    path_follower path_follower;
    ElapsedTime timer = new ElapsedTime();
    Pose2d poseToHold = new Pose2d();
    Pose2d sample1 = new Pose2d(47.7, 34.5, -Math.PI / 2);
    Pose2d sample2 = new Pose2d(57, 33.4, -Math.PI / 2);
    Pose2d sample3 = new Pose2d(58.7, 31.7, -Math.PI / 2 + Math.toRadians(42.3));

    //SampleDetectionProcessor sampleDetection = new SampleDetectionProcessor();

    @Override
    public void runOpMode() throws InterruptedException {
        //logger.init();
        /*Smotritel smotritel = new Smotritel(hardwareMap);
        smotritel.stopStream();*/
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
            poseToHold = path_follower.goToPosWithArmToBasket(53.356, 49.35, -Math.PI * 3 / 4);
            //logger.writeLn("----------STARTING SCORING PRELOAD------------");
            scoreHighBasket(sample1);

            path_follower.goToPosWithArm(sample1.getX(), sample1.getY(), sample1.getHeading()); /* sample 1 */
            //path_follower.goToPosUnsafe(51.4, 51.4, -Math.PI / 2 - Math.toRadians(11.5)); /* sample 1 */
            waitArmRotation();
            waitArmExtension();
            takeSample(false);

            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            module_master.arm.setExtension(arm.extension.EXTENDED);
            poseToHold = path_follower.goToPosWithArmToBasket(53.4, 53.4, -Math.PI * 3 / 4);
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
            poseToHold = path_follower.goToPosWithArmToBasket(54, 54, -Math.PI * 3 / 4 + Math.toRadians(6));
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

            //module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchHalfDown();
            path_follower.velocity_calculator.setThirdSampleRotationCoeffs();
            module_master.arm.setExtension(arm.extension.EXTENDED);
            poseToHold = path_follower.goToPosWithArmToBasket(49.2, 49.2, -Math.PI * 3 / 4 - Math.toRadians(8));
            path_follower.velocity_calculator.setDefaultRotationCoeffs();
            //logger.writeLn("----------STARTING SCORING THIRD------------");
            scoreHighBasket();

            /* go to dobor1 */
            /*
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 54,-Math.PI, new double[]{0.2, 0.2, 0.34, 0.34}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CAMERA, SET_ROTATION_CAMERA});
            if (!takeFromSubmersible(smotritel, 1))
                if (!takeFromSubmersible(smotritel, -1))
                    instapark();
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 90, 42, -Math.PI, new double[]{0.2, 0.2, 0.8}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            path_follower.goToPosVeryUnsafe(22.2, 9, -Math.PI);

            module_master.differential.pitchDown();
            module_master.differential.rollHalfLeft();
            delay(200);
            takeSample(true);
            module_master.differential.rollDefault();
            module_master.differential.setPitch(135);
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor1 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.2, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(52.5, 50.1, -Math.PI * 3 / 4 + Math.toRadians(10));
            waitArmExtension();
            scoreHighBasket();
            /* go to dobor2 */
            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 84,39, -Math.PI, new double[]{0.2, 0.2, 0.34, 0.34}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CAMERA, SET_ROTATION_CAMERA});
            /*if (!takeFromSubmersible(smotritel, 2))
                if (!takeFromSubmersible(smotritel, -1))
                    instapark();*/
            module_master.arm.setExtension(arm.extension.CLOSED_AUTO);
            /* go to score dobor2 */
            path_follower.followTrajectoryBackwards(curves[5], new double[]{0.02, 0.06, 0.2, 0.3}, new int[]{SET_EXTENSION_CLOSED, SET_ROTATION_LIFT, SET_EXTENSION_LIFT, PITCH_DOWN});
            path_follower.goToPosWithArmToBasket(52.2, 50.8, -Math.PI * 3 / 4 + Math.toRadians(8));
            waitArmExtension();
            scoreHighBasket();

            path_follower.followTrajectoryForwardsPercentageHypeAngleControl(curves[4], 94, 54, -Math.PI, new double[]{0.2, 0.2, 0.65}, new int[]{SET_ROTATION_FRONT, PITCH_FRONT, SET_EXTENSION_CHAMBER});
            module_master.arm.setRotation(arm.rotation.CHAMBER);
            path_follower.goToPosVeryUnsafe(23, 9, -Math.PI);

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

        module_master.differential.closeClawSilno();
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
        module_master.differential.pitchScoringBasket();
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

    private boolean takeFromSubmersible(Smotritel smotritel, int call){
        smotritel.startStream();
        //smotritel.startDashboardStream();

        module_master.arm.setRotation(arm.rotation.CAMERA);
        module_master.arm.setExtension(arm.extension.CAMERA);
        module_master.differential.openClaw();
        module_master.differential.setRoll(-10);
        module_master.differential.setPitch(73);
        module_master.differential.update();

        if (call == -1)
            ;
        else if (call == 1)
            path_follower.goToPosVeryUnsafe(22.2, 7, -Math.PI);
        else if (call == 2)
            path_follower.goToPosVeryUnsafe(22.2, 9, -Math.PI);
        else
            path_follower.goToPosVeryUnsafe(22.2, 7.5, -Math.PI);
        while ((!module_master.arm.extensionReached() || !module_master.arm.rotationReached()) && opModeIsActive())
            module_master.update(dataStorage.telemetry);

        dataStorage.updateData();
        double yOffset = 0;
        int xOffset = 0;
        Sample nearest = smotritel.getNearestSample();
        telemetry.addData("area", nearest.getArea());
        telemetry.update();
        double angle = nearest.getAngle();

        if (nearest.getColor() == Sample.SampleColor.UNDETECTED) {
            if (call == -1)
                path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY - 4, dataStorage.RobotWorldHeading);
            else
                path_follower.goToPos(dataStorage.RobotWorldX + 4, dataStorage.RobotWorldY, dataStorage.RobotWorldHeading);
            //smotritel.stopStream();
            //smotritel.startDashboardStream();
            return false;
        }
        else {
            if (nearest.getArea() > 10000) {
                smotritel.stopStream();
                //smotritel.stopDashboardStream();
            }
            yOffset = BlackPipeline.pixelToInchesY(nearest.getCenter().y);
            xOffset = BlackPipeline.pixelToTicks(nearest.getCenter().x);
            if (nearest.getArea() > 10000) {
                module_master.arm.ROTATION_PIDF = new PIDFCoefficients(0.0026, 0, 0.0019, 0);
                module_master.arm.pidExtend(xOffset);
                module_master.arm.setRotation(arm.rotation.FRONT);
            }
            path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY + yOffset, dataStorage.RobotWorldHeading);

            if (nearest.getArea() <= 10000) {
                nearest = smotritel.getNearestSample();
                telemetry.addData("area", nearest.getArea());
                telemetry.update();
                smotritel.stopStream();
                //smotritel.startDashboardStream();
                angle = nearest.getAngle();

                yOffset = BlackPipeline.pixelToInchesY(nearest.getCenter().y);
                xOffset = BlackPipeline.pixelToTicks(nearest.getCenter().x);
                module_master.arm.ROTATION_PIDF = new PIDFCoefficients(0.0026, 0, 0.0019, 0);
                module_master.arm.pidExtend(xOffset);
                module_master.arm.setRotation(arm.rotation.FRONT);
                path_follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY + yOffset, dataStorage.RobotWorldHeading);
            }
        }
        while ((!module_master.arm.extensionReached() || !module_master.arm.rotationReached()) && opModeIsActive())
            module_master.update(dataStorage.telemetry);
        module_master.differential.pitchDown();
        delay(300);
        module_master.differential.setRoll(differential.geomToDifAngle(angle));
        module_master.differential.update();
        delay(300);

        module_master.differential.closeClaw();
        delay(200);
        module_master.differential.setPitch(135);
        module_master.differential.rollDefault();
        module_master.differential.update();
        module_master.arm.ROTATION_PIDF = new PIDFCoefficients(0.0026, 0, 0.0019, -0.0024);
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
        module_master.arm.setExtension(arm.extension.HIGH_CHAMBER);
        module_master.arm.setRotation(arm.rotation.CHAMBER);
        waitArmRotation();

        while (opModeIsActive()) {
            module_master.arm.update();
            module_master.arm.manuallyExtend(0);
        }
        robot.stop();
        transfer.angle = dataStorage.RobotWorldHeading;
    }
}