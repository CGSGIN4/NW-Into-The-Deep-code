package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.FRONTAL_EXTENSION;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.velocity_calculator;
import org.firstinspires.ftc.teamcode.utils.painter;

@TeleOp
public class armTest extends LinearOpMode {
    Robot robot;

    Vector2d gamepad;
    arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new arm(hardwareMap);
        //robot = new Robot(hardwareMap);
        //robot.init();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad PreviousGamepad1 = new Gamepad();
        Gamepad PreviousGamepad2 = new Gamepad();

        ElapsedTime timer = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            PreviousGamepad1.copy(currentGamepad1);
            PreviousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            arm.update();
            if (gamepad1.a)
            {
                arm.setRotation(LIFT);
            }

            if (gamepad1.b)
            {
                arm.setRotation(FRONT);
            }

            if (gamepad1.x)
                arm.setExtension(FRONTAL_EXTENSION);
            if (gamepad1.y)
                arm.setExtension(CLOSED);

            if (Math.abs(gamepad1.left_stick_y) > 0.01)
                arm.manuallyExtend(-gamepad1.left_stick_y);
            else
                arm.extensionMotor.setPower(0);

            packet.put("target pos", arm.targetRotationPos);
            packet.put("current pos", arm.rotationMotor.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("rotation pos", arm.rotationMotor.getCurrentPosition());
            telemetry.addData("extension pos", arm.extensionMotor.getCurrentPosition());
            telemetry.addData("extension power", -gamepad1.left_stick_y / 10);
            telemetry.addData("rotation power", arm.pidCalculateRotationPower(arm.targetRotationPos));
            telemetry.addData("rotation state", arm.rotationState.toString());
            telemetry.addData("extension state", arm.extensionState.toString());
            telemetry.addData("rotation angle", arm.getRotationAngle());
            telemetry.addData("extension length", arm.getExtensionLength());
            telemetry.addData("hold power", 0.03 * Math.sin(Math.toRadians(arm.rotationAngle)) * arm.extensionLen);
            telemetry.addData("cycle time", timer.milliseconds());
            telemetry.update();
            timer.reset();
            /*
            maxVel = Math.max(maxVel, dataStorage.RobotVelocity.norm());
            telemetry.addData("velocity", dataStorage.RobotVelocity.norm());
            telemetry.addData("max vel", maxVel);
            telemetry.update();
            */
        }
    }
}
