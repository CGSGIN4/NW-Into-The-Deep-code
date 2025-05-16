package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.data.dataStorage.timer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.vision.LLSmotritel;

import java.util.function.BooleanSupplier;

@TeleOp
public class LLGrabTest extends LinearOpMode {
    ElapsedTime delayTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.init();

        dataStorage.init(bot.drive, telemetry, this);
        dataStorage.opModeIsAutonomous = false;
        bot.drive.setPoseEstimate(new Pose2d(39.9, 64.93, Math.PI));
        path_follower follower = new path_follower(bot.drivetrain);

        ElapsedTime timer = new ElapsedTime();
        LLSmotritel smotritel = new LLSmotritel(hardwareMap);

        waitForStart();

        module_master.arm.stop();
        module_master.differential.setPitch(180);
        smotritel.startStreaming();
        module_master.arm.setRotation(org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT);
        module_master.differential.openClaw();
        while (opModeIsActive())
        {
            module_master.differential.setPitch(90);
            module_master.differential.setRoll(-11);
            module_master.differential.update();
            module_master.arm.update(dataStorage.telemetry);
            dataStorage.updateData();
            Pose2d offsets = smotritel.getSampleOffsets(0);

            dataStorage.telemetry.addData("cam x offset", offsets.getX());
            dataStorage.telemetry.addData("cam y offset", offsets.getY());
            dataStorage.telemetry.addData("sample rotation", offsets.getHeading());
            dataStorage.telemetry.addData("ticks", smotritel.getTicks(offsets));
            dataStorage.telemetry.addData("actual ticks", module_master.arm.extensionMotor.getCurrentPosition());
            dataStorage.telemetry.addData("robot heading", dataStorage.RobotWorldHeading);
            dataStorage.telemetry.addData("target heading", Math.PI + Math.toRadians(offsets.getX()));
            dataStorage.telemetry.addData("heading err", Math.abs(dataStorage.RobotWorldHeading - (Math.PI + Math.toRadians(offsets.getX()))));
            dataStorage.telemetry.addData("y offset in inches", smotritel.getTranslationalOffset(offsets));
            if (gamepad1.a && offsets.getX() != -100)
            {
                boolean smestilsya = false;
                boolean nevidel = false;
                bot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.45, 0).rotated(Math.toRadians(90)), 0);
                dataStorage.updateData();
                delay(100, "Holyshit");
                while (dataStorage.RobotVelocity.norm() > 2)
                    delay(1, "Fuckingshit");

                dataStorage.photoVel = dataStorage.RobotVelocity.norm();
                while (!smotritel.startSnapshot())
                {
                    delay(1);
                    dataStorage.telemetry.addData("alllo", "alllooo");
                }
                offsets = smotritel.getSampleOffsets(0);
                Pose2d snapshotPos = new Pose2d(dataStorage.RobotPose, dataStorage.RobotWorldHeading);

                if ((offsets.getX() == -100 || Math.abs(LLSmotritel.getTicks(offsets)) > module_master.arm.EXTENSION_FRONT_MAX) && opModeIsActive())
                {
                    dataStorage.updateData();
                    module_master.arm.update(dataStorage.telemetry);
                    follower.goToPos(snapshotPos.getX(), dataStorage.RobotWorldY - 7, -Math.PI);
                    while (!smotritel.startSnapshot()) delay(1);
                    offsets = smotritel.getSampleOffsets(0);
                    nevidel = true;
                }
                if (nevidel) {
                    if (offsets.getX() == -100) {
                        continue;
                    }
                    bot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.45, 0).rotated(Math.toRadians(90)), 0);
                    delay(100);
                    while (dataStorage.RobotVelocity.norm() > 2)
                        delay(1);
                    while (!smotritel.startSnapshot()) {
                        delay(1);
                        dataStorage.telemetry.addData("alllo", "alllooo");
                    }
                    offsets = smotritel.getSampleOffsets(0);
                    dataStorage.telemetry.addData("angle", dataStorage.RobotWorldHeading);
                }
                Pose2d rememberedOffsets = offsets;

                while (Math.abs(offsets.getX()) > 15 && opModeIsActive()) {
                    if (offsets.getX() == -100) {
                        offsets = rememberedOffsets;
                        break;
                    }
                    dataStorage.updateData();
                    follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets), -Math.PI);
                    while (!smotritel.startSnapshot()) {
                        delay(1);
                        dataStorage.telemetry.addData("alllo", "alllooo");
                    }
                    offsets = smotritel.getSampleOffsets(0);
                    smestilsya = true;
                }
                if (smestilsya) {
                    bot.drivetrain.applyVectorFieldCentric(new Vector2d(-0.45, 0).rotated(Math.toRadians(90)), 0);
                    delay(100);
                    while (dataStorage.RobotVelocity.norm() > 2)
                        delay(1);
                    while (!smotritel.startSnapshot()) {
                        delay(1);
                        dataStorage.telemetry.addData("alllo", "alllooo");
                    }
                    offsets = smotritel.getSampleOffsets(0);
                }

                module_master.differential.setPitch(0);
                module_master.differential.setRoll(offsets.getHeading());
                module_master.differential.update();

                module_master.arm.setRotation(org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT);
                module_master.arm.pidExtend(LLSmotritel.getTicks(offsets));
                //dataStorage.drive.setPoseEstimate(new Pose2d(22, dataStorage.RobotWorldY, dataStorage.RobotWorldHeading));
                dataStorage.updateData();
                if (dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets) < -8)
                    continue;
                follower.goToPosPreciseMTI(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets), -Math.PI);

                while (Math.abs(module_master.arm.extensionMotor.getCurrentPosition() - LLSmotritel.getTicks(offsets)) + module_master.arm.offset > 13 && opModeIsActive()) {
                    if (module_master.arm.rotationReached())
                        module_master.arm.pidExtend(LLSmotritel.getTicks(offsets));
                    module_master.arm.update(dataStorage.telemetry);
                }
                waitArmRotation();
                waitArmExtension();
                delay(0);
                module_master.differential.closeClaw();
                delay(125);
                module_master.differential.setPitch(70);
                module_master.differential.setRoll(-11);
                module_master.differential.update();
                module_master.arm.setExtension(arm.extension.CLOSED);
            }
            else
                bot.drivetrain.applyVector(new Vector2d(0, 0), 0);
            if (gamepad1.b)
            {
                module_master.differential.openClaw();
                module_master.differential.setPitch(160);
                module_master.differential.setRoll(-11);
                module_master.differential.update();
                module_master.arm.setRotation(org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.CHAMBER);
            }
        }
    }
    private void delay(long milliseconds) {
        delayTimer.reset();
        module_master.arm.update(dataStorage.telemetry);
        dataStorage.updateData();
        while (delayTimer.milliseconds() < milliseconds && opModeIsActive()) {
            module_master.arm.update(dataStorage.telemetry);
            dataStorage.updateData();
        }
    }
    private void delay(long milliseconds, String tele) {
        delayTimer.reset();
        module_master.arm.update(dataStorage.telemetry);
        dataStorage.updateData();
        while (delayTimer.milliseconds() < milliseconds && opModeIsActive()) {
            dataStorage.telemetry.addLine(tele);
            module_master.arm.update(dataStorage.telemetry);
            dataStorage.updateData();
        }
    }

    private void waitArmExtension(){
        waitForCondition(() -> module_master.arm.extensionReached());
    }

    private void waitArmRotation(){
        waitForCondition(() -> module_master.arm.rotationReached());
    }
    private void waitForCondition(BooleanSupplier condition) {
        delayTimer.reset();
        while (!condition.getAsBoolean() && opModeIsActive() && timer.milliseconds() < 5000) {
            module_master.update(dataStorage.telemetry);
            dataStorage.updateData();
        }
    }
}
