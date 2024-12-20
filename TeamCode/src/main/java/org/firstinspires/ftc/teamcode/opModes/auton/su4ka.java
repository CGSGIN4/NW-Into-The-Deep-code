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
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchDown();
            path_follower.followTrajectoryBreak(curves[0], -Math.PI * 3 / 4);
            waitArmRotation();

            setExtensionAndWait(arm.extension.EXTENDED);
            module_master.arm.setRotation(arm.rotation.BACK_HANG1);
            module_master.differential.pitchScoringBasket();
            delay(500);

            module_master.differential.openClaw();
            delay(200);
            module_master.arm.setRotation(arm.rotation.LIFT);

            module_master.differential.pitchHalfDown();
            delay(300);

            setExtensionAndWait(arm.extension.CLOSED);

            module_master.arm.setRotation(arm.rotation.FRONT);
            module_master.differential.pitchForward();
            path_follower.goToPos(48.3, 51, -Math.PI / 2);
            //path_follower.goToPosUnsafe(51.4, 51.4, -Math.PI / 2 - Math.toRadians(11.5)); /* sample 1 */
            waitArmRotation();
            setExtensionAndWait(arm.extension.YELLOW_1);
            takeSample(false);
            setExtensionAndWait(arm.extension.CLOSED);

            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchDown();
            path_follower.goToPosUnsafe(51.4, 51.4, -Math.PI * 3 / 4);

            /* start rotating for scoring yellow 1 */
            waitArmRotation();

            setExtensionAndWait(arm.extension.EXTENDED);
            module_master.arm.setRotation(arm.rotation.BACK_HANG1);
            module_master.differential.pitchScoringBasket();
            delay(500);

            module_master.differential.openClaw();
            delay(200);
            module_master.arm.setRotation(arm.rotation.LIFT);

            module_master.differential.pitchHalfDown();
            delay(300);

            setExtensionAndWait(arm.extension.CLOSED);

            module_master.arm.setRotation(arm.rotation.FRONT);
            module_master.differential.pitchForward();
            path_follower.goToPos(59, 51.4, -Math.PI / 2); /* sample 2 */
            waitArmRotation();
            module_master.arm.setExtension(arm.extension.YELLOW_2);
            waitArmExtension();

            takeSample(false);
            setExtensionAndWait(arm.extension.CLOSED);

            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchDown();
            path_follower.goToPosUnsafe(51.4, 51.4, -Math.PI * 3 / 4);

            /* start rotating for scoring yellow 2 */
            waitArmRotation();

            setExtensionAndWait(arm.extension.EXTENDED);
            module_master.arm.setRotation(arm.rotation.BACK_HANG1);
            module_master.differential.pitchScoringBasket();
            delay(500);

            module_master.differential.openClaw();
            delay(200);
            module_master.arm.setRotation(arm.rotation.LIFT);

            module_master.differential.pitchHalfDown();
            delay(300);

            setExtensionAndWait(arm.extension.CLOSED);

            module_master.arm.setRotation(arm.rotation.FRONT);
            module_master.differential.pitchForward();
            path_follower.goToPos(59, 47, -Math.PI / 2 + Math.toRadians(23.3));
            waitArmRotation();

            /* intaking yellow 3 */
            module_master.differential.setRoll(23);
            module_master.differential.update();
            setExtensionAndWait(arm.extension.YELLOW_3);
            takeSample(true);
            setExtensionAndWait(arm.extension.CLOSED);
            module_master.differential.rollDefault();

            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.differential.pitchDown();
            path_follower.goToPosUnsafe(51.4, 51.4, -Math.PI * 3 / 4);

            /* start rotating for scoring yellow 3 */
            waitArmRotation();

            setExtensionAndWait(arm.extension.EXTENDED);
            module_master.arm.setRotation(arm.rotation.BACK_HANG1);
            module_master.differential.pitchScoringBasket();
            delay(500);

            module_master.differential.openClaw();
            delay(200);
            module_master.arm.setRotation(arm.rotation.LIFT);

            module_master.differential.pitchHalfDown();
            delay(300);

            setExtensionAndWait(arm.extension.CLOSED);
            module_master.arm.setRotation(arm.rotation.FRONT);
            waitArmRotation();
            setExtensionAndWait(arm.extension.CLOSED);

            robot.stop();
            module_master.stop(dataStorage.telemetry);
            transfer.armExtensionPos = module_master.arm.extensionMotor.getCurrentPosition();
            transfer.armRotationPos = module_master.arm.rotationMotor.getCurrentPosition();

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
    private void takeSample(boolean third){
        delay(300);
        if (!third) {
            module_master.differential.setPitch(37);
            module_master.differential.update();
        }
        else {
            module_master.differential.setPitch(35);
            module_master.differential.update();
        }
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

        module_master.arm.setRotation(arm.rotation.BACK_HANG1);

        /* SCORE SAMPLE */
        module_master.differential.pitchScoringBasket();
        delay(300);

        //waitArmRotation();
        module_master.differential.openClaw();
        delay(300);

        module_master.arm.setRotation(arm.rotation.LIFT);

        module_master.differential.pitchHalfDown();
        delay(300);

        /* FOLD */
        module_master.doAction(SET_EXTENSION_CLOSED);
    }
}