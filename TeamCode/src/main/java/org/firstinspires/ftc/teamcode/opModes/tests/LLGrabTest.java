package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.vision.LLSmotritel;

@TeleOp
public class LLGrabTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        differential differential = new differential(hardwareMap);
        bot.init();

        dataStorage.init(bot.drive, telemetry, this);
        dataStorage.opModeIsAutonomous = false;
        arm arm = new arm(hardwareMap, true);
        bot.drive.setPoseEstimate(new Pose2d(39.9, 64.93, Math.PI));
        path_follower follower = new path_follower(bot.drivetrain);

        ElapsedTime timer = new ElapsedTime();
        LLSmotritel smotritel = new LLSmotritel(hardwareMap);

        waitForStart();

        arm.stop();
        differential.setPitch(180);
        smotritel.startStreaming();
        arm.setRotation(org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT);
        differential.openClaw();
        while (opModeIsActive())
        {
            arm.update();
            Pose2d offsets = smotritel.getSampleOffsets(0);

            telemetry.addData("cam x offset", offsets.getX());
            telemetry.addData("cam y offset", offsets.getY());
            telemetry.addData("sample rotation", offsets.getHeading());
            telemetry.addData("ticks", smotritel.getTicks(offsets));
            telemetry.addData("actual ticks", arm.extensionMotor.getCurrentPosition());
            telemetry.addData("robot heading", dataStorage.RobotWorldHeading);
            telemetry.addData("target heading", Math.PI + Math.toRadians(offsets.getX()));
            telemetry.addData("heading err", Math.abs(dataStorage.RobotWorldHeading - (Math.PI + Math.toRadians(offsets.getX()))));
            telemetry.addData("y offset in inches", smotritel.getTranslationalOffset(offsets));
            telemetry.update();
            if (gamepad1.a && offsets.getX() != -100)
            {
                dataStorage.updateData();
                offsets = smotritel.getSampleOffsets(0);
                while (Math.abs(offsets.getX()) > 15) {
                    dataStorage.updateData();
                    smotritel.startSnapshot();
                    follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets), -Math.PI);
                    offsets = smotritel.getSampleOffsets(0);
                }
                follower.goToPosPrecise(dataStorage.RobotWorldX, dataStorage.RobotWorldY + smotritel.getTranslationalOffset(offsets), -Math.PI);

                telemetry.addData("cam x offset", offsets.getX());
                telemetry.addData("cam y offset", offsets.getY());
                telemetry.addData("sample rotation", offsets.getHeading());
                telemetry.addData("ticks", smotritel.getTicks(offsets));
                telemetry.addData("actual ticks", arm.extensionMotor.getCurrentPosition());
                telemetry.addData("robot heading", dataStorage.RobotWorldHeading);
                telemetry.addData("target heading", Math.PI + Math.toRadians(offsets.getX()));
                telemetry.addData("heading err", Math.abs(dataStorage.RobotWorldHeading - (Math.PI + Math.toRadians(offsets.getX()))));
                telemetry.addData("y offset in inches", smotritel.getTranslationalOffset(offsets));
                telemetry.update();

                if (offsets.getX() == -100)
                    continue;

                arm.setRotation(org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT);
                while (Math.abs(arm.extensionMotor.getCurrentPosition() - smotritel.getTicks(offsets)) > 13) {
                    telemetry.addData("cam x offset", offsets.getX());
                    telemetry.addData("cam y offset", offsets.getY());
                    telemetry.addData("sample rotation", offsets.getHeading());
                    telemetry.addData("ticks", smotritel.getTicks(offsets));
                    telemetry.addData("actual ticks", arm.extensionMotor.getCurrentPosition());
                    telemetry.addData("robot heading", dataStorage.RobotWorldHeading);
                    telemetry.addData("target heading", Math.PI + Math.toRadians(offsets.getX()));
                    telemetry.addData("heading err", Math.abs(dataStorage.RobotWorldHeading - (Math.PI + Math.toRadians(offsets.getX()))));
                    telemetry.addData("y offset in inches", smotritel.getTranslationalOffset(offsets));
                    telemetry.update();

                    if (arm.rotationReached())
                        arm.pidExtend(smotritel.getTicks(offsets));
                    arm.update();
                }
                differential.setPitch(0);
                differential.update();
                timer.reset();
                while (timer.milliseconds() < 300) arm.update();
                differential.setRoll(offsets.getHeading());
                differential.update();
                timer.reset();
                while (timer.milliseconds() < 300) arm.update();
                differential.closeClaw();
                timer.reset();
                while (timer.milliseconds() < 300) arm.update();
                differential.setPitch(135);
                differential.update();
                arm.setExtension(org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED);

                bot.drivetrain.applyVector(new Vector2d(0, 0), 0);
            }
            else
                bot.drivetrain.applyVector(new Vector2d(0, 0), 0);
            if (gamepad1.b)
            {
                differential.openClaw();
                differential.setPitch(160);
                differential.setRoll(-11);
                differential.update();
                arm.setRotation(org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT);
            }
        }
    }
}
