package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.FRONTAL_EXTENSION;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.HIGH_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.LOW_CHAMBER;
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

            if (gamepad1.dpad_right)
                arm.setExtension(LOW_CHAMBER);
            if (gamepad1.dpad_left)
                arm.setExtension(HIGH_CHAMBER);
            if (gamepad1.dpad_down)
                arm.setExtension(FRONTAL_EXTENSION);
            if (gamepad1.dpad_up)
                arm.setExtension(EXTENDED);


            if (gamepad1.y)
                arm.setExtension(CLOSED);


            /*
            if (Math.abs(gamepad1.right_stick_y) > 0.01)
                arm.manuallyRotate(-gamepad1.right_stick_y);
            else
                arm.rotationMotor.setPower(0);
            */


            packet.put("target pos", arm.targetRotationPos);
            packet.put("current pos", arm.rotationMotor.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("rotation pos", arm.rotationMotor.getCurrentPosition());
            telemetry.addData("gamepad", gamepad1.right_stick_y);
            telemetry.addData("extension pos", arm.extensionMotor.getCurrentPosition());
            telemetry.addData("extension power", -gamepad1.left_stick_y / 10);
            telemetry.addData("rotation power", arm.pidCalculateRotationPower(arm.targetRotationPos));
            telemetry.addData("rotation state", arm.rotationState.toString());
            telemetry.addData("extension state", arm.extensionState.toString());
            telemetry.addData("rotation angle", arm.getRotationAngle());
            telemetry.addData("extension length", arm.getExtensionLength());
            telemetry.addData("hold power", 0.03 * Math.sin(Math.toRadians(arm.rotationAngle)) * arm.extensionLen);
            telemetry.addData("cog", (arm.m_1 * Math.sqrt(arm.l_1 * arm.l_1 + arm.h_1 * arm.h_1) + arm.m_2 * Math.sqrt(arm.l_2 * arm.l_2 + arm.h_2 * arm.h_2) + arm.m_3 * Math.sqrt(arm.l_3 * arm.l_3 + arm.h_3 * arm.h_3) + arm.m_4 * Math.sqrt(arm.l_4 * arm.l_4 + arm.h_4 * arm.h_4) + arm.m_5 * Math.sqrt(arm.l_5 * arm.l_5 + arm.h_5 * arm.h_5)) / (arm.m_1 + arm.m_2 + arm.m_3 + arm.m_4 + arm.m_5));
            telemetry.addData("cycle time", timer.milliseconds());
            telemetry.update();
            timer.reset();
            /*
            maxVel = Math.max(maxVel, dataStorage.RobotVelocity.norm());
            telemetry.addData("cog", (arm.m_1 * arm.l_1 + arm.m_2 * arm.l_2 + arm.m_3 * arm.l_3 + arm.m_4 * arm.l_4 + arm.m_5 * arm.l_5) / (arm.m_1 + arm.m_2 + arm.m_3 + arm.m_4 + arm.m_5));
            telemetry.addData("max vel", maxVel);
            telemetry.update();
            */
        }
    }
}
