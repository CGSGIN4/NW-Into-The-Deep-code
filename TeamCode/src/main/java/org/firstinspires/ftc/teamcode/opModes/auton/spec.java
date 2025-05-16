package org.firstinspires.ftc.teamcode.opModes.auton;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENSION_SPEC_PREP;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENSION_SPEC_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENSION_TAKE_SPEC_FROM_FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.TAKE_SPEC;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLOSE_CLAW_VERY_SILNO;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_SCORING_BASKET;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.PITCH_UP;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_EXTENSION_PREP_SPEC;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.SET_ROTATION_FRONT;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.data.transfer;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.hang;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.vision.LLSmotritel;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.IOException;
import java.util.Vector;
import java.util.function.BooleanSupplier;

@Autonomous
public class spec extends LinearOpMode {

    Robot robot;
    path_follower path_follower;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime takeSpecTimer = new ElapsedTime();
    private Pose2d poseToHold = new Pose2d(-100, 0, 0);
    Pose2d takeFromFloorPos = new Pose2d(-28, 52, Math.PI * 3 / 4);

    enum states{

    }

    states state;
    @Override
    public void runOpMode() throws InterruptedException {
        LLSmotritel smotritel = new LLSmotritel(hardwareMap, 0);
        robot = new Robot(hardwareMap);
        robot.init();
        parser parser = new parser("spec");
        dataStorage.init(robot.drive, telemetry, this);
        path_follower = new path_follower(robot.drivetrain);
        robot.drive.setPoseEstimate(new Pose2d(-7.0665, 63.9, Math.PI / 2));

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
            module_master.differential.pitchUp();
            module_master.arm.setRotation(arm.rotation.LIFT);
            module_master.arm.setExtension(arm.extension.EXTENSION_SPEC_PREP);
            module_master.hang.state = hang.states.EXTENDED_SPEC;
            path_follower.velocity_calculator.p_trans_coef = 0.055;
            path_follower.velocity_calculator.d_trans_coef = 0.14;
            path_follower.velocity_calculator.i_trans_coef = 0.005;
            path_follower.velocity_calculator.P_ROTATION_COEF = 0.5;
            path_follower.velocity_calculator.D_ROTATION_COEF = 1.4;
            path_follower.velocity_calculator.I_ROTATION_COEF = 0.08;
            path_follower.goToPosSpec(-1.0655, 38.3, Math.PI / 2);
            scoreSpec();

            path_follower.followTrajectory(curves[0], Math.PI / 2);
            module_master.differential.pitchUp();
            path_follower.followTrajectoryNonStop(curves[1], Math.PI / 2);
            path_follower.followTrajectoryNonStop(curves[2], Math.PI / 2);
            path_follower.followTrajectory(curves[3], Math.PI / 2);
            module_master.arm.setRotation(TAKE_SPEC);
            module_master.differential.setPitch(60);
            module_master.differential.update();
            path_follower.followTrajectoryNonStop(curves[4], Math.PI / 2);
            path_follower.followTrajectory(curves[5], Math.PI / 2);


            path_follower.velocity_calculator.d_trans_coef *= 1.13;
            path_follower.goToPos(-62.5, 60.2, Math.PI / 2);
            path_follower.velocity_calculator.d_trans_coef /= 1.13;
            module_master.differential.closeClawSilno();
            delay(150);
            module_master.differential.pitchUp();
            delay(20);
            module_master.arm.setRotation(LIFT);
            path_follower.followTrajectoryNonStop(curves[7], Math.PI / 2, new double[]{0.9}, new int[]{SET_EXTENSION_PREP_SPEC});
            module_master.differential.closeClawVerySilno();
            path_follower.goToPosSpec(1.0655, 38.3, Math.PI / 2);
            scoreSpec();



            path_follower.velocity_calculator.p_trans_coef *= 2.5;
            path_follower.velocity_calculator.d_trans_coef *= 1.4;
            path_follower.velocity_calculator.P_ROTATION_COEF *= 1.45;
            path_follower.goToPosWithArmSpec(-28, 52, Math.PI * 3 / 4);
            path_follower.velocity_calculator.P_ROTATION_COEF /= 1.45;
            module_master.arm.setExtension(EXTENSION_TAKE_SPEC_FROM_FLOOR);
            waitArmExtension();

            module_master.differential.closeClawVerySilno();
            delay(200);
            module_master.arm.setExtension(CLOSED);
            module_master.arm.setRotation(LIFT);
            module_master.differential.pitchUp();

            path_follower.followTrajectoryNonStop(curves[8], Math.PI / 2, new double[]{0.2}, new int[]{SET_EXTENSION_PREP_SPEC});
            module_master.differential.closeClawVerySilno();
            path_follower.goToPosWithArmSpecToChamber(-5.0655, 38.8, Math.PI / 2);
            scoreSpec();


            path_follower.velocity_calculator.P_ROTATION_COEF *= 1.45;
            path_follower.goToPosWithArmSpec(-28, 52, Math.PI * 3 / 4);
            path_follower.velocity_calculator.P_ROTATION_COEF /= 1.45;
            module_master.arm.setExtension(EXTENSION_TAKE_SPEC_FROM_FLOOR);
            waitArmExtension();

            module_master.differential.closeClawVerySilno();
            delay(200);
            module_master.arm.setExtension(CLOSED);
            module_master.arm.setRotation(LIFT);
            module_master.differential.pitchUp();

            path_follower.followTrajectoryNonStop(curves[8], Math.PI / 2, new double[]{0.2}, new int[]{SET_EXTENSION_PREP_SPEC});
            module_master.differential.closeClawVerySilno();
            path_follower.goToPosWithArmSpecToChamber(-7.0655, 39.7, Math.PI / 2);
            scoreSpec();



            path_follower.velocity_calculator.P_ROTATION_COEF *= 1.45;
            path_follower.goToPosWithArmSpec(-28, 52, Math.PI * 3 / 4);
            path_follower.velocity_calculator.P_ROTATION_COEF /= 1.45;
            module_master.arm.setExtension(EXTENSION_TAKE_SPEC_FROM_FLOOR);
            waitArmExtension();

            module_master.differential.closeClawVerySilno();
            delay(200);
            module_master.arm.setExtension(CLOSED);
            module_master.arm.setRotation(LIFT);
            module_master.differential.pitchUp();

            path_follower.followTrajectory(curves[8], Math.PI / 2, 70, new double[]{0.2}, new int[]{SET_EXTENSION_PREP_SPEC});
            module_master.differential.closeClawVerySilno();
            path_follower.goToPosWithArmSpecToChamber(-9.0655, 39.4, Math.PI / 2);
            scoreSpec();

            path_follower.velocity_calculator.P_ROTATION_COEF *= 1.45;
            path_follower.goToPosWithArmSpec(-32, 54, Math.PI * 3 / 4);
            path_follower.velocity_calculator.P_ROTATION_COEF /= 1.45;
            module_master.arm.setExtension(EXTENSION_TAKE_SPEC_FROM_FLOOR);


            robot.stop();
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

    void scoreSpec(boolean prestart){
        //poseToHold = takeFromFloorPos;
        module_master.arm.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        module_master.differential.closeClawVerySilno();
        module_master.arm.setExtension(EXTENSION_SPEC_SCORE);
        while ((module_master.arm.extensionMotor.getCurrentPosition() + module_master.arm.offset < 800 || module_master.arm.extensionMotor.getVelocity() > 20) && opModeIsActive()) {
            dataStorage.telemetry.addData("motor vel", module_master.arm.extensionMotor.getVelocity());
            delay(1);
        }

        module_master.differential.openClaw();
        module_master.arm.setExtension(CLOSED);
        module_master.arm.setRotation(FRONT);
        module_master.differential.setPitch(35);
        module_master.differential.update();
        poseToHold = new Pose2d(-100, 0,0);
    }

    void scoreSpec(){
        scoreSpec(false);
    }
    private void waitArmExtension(){
        waitForCondition(() -> module_master.arm.extensionReached());
    }

    private void waitArmRotation(){
        waitForCondition(() -> module_master.arm.rotationReached());
    }

    private boolean headingAroundPi(double threshold)
    {
        return (Math.abs(dataStorage.RobotWorldHeading + Math.PI) < threshold || Math.abs(dataStorage.RobotWorldHeading + Math.PI) - 6.28 < threshold);
    }
}