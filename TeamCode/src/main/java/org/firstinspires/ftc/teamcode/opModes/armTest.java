package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.FRONTAL_EXTENSION;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.HIGH_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.LOW_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.BACK;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@Config
@TeleOp
public class armTest extends LinearOpMode {
    Robot robot;

    Vector2d gamepad;
    arm arm;
    public static double p = 0, i = 0, d = 0, f = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new arm(hardwareMap);
        //robot = new Robot(hardwareMap);
        //robot.init();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        ElapsedTime timer = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        MultipleTelemetry tele = new MultipleTelemetry(dashboardTelemetry, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            arm.update();

            if (currentGamepad1.a && !previousGamepad1.a)
            {
                if (arm.rotationState == LIFT)
                    arm.setRotation(BACK);
<<<<<<< Updated upstream
                else
                    arm.setRotation(LIFT);
=======
                    //tele.addData("rotation power", arm.setRotation(BACK));
                else if (arm.rotationState == CHAMBER)
                    arm.setRotation(LIFT);
                else
                    arm.setRotation(CHAMBER);
                    //tele.addData("rotation power", arm.setRotation(LIFT));
>>>>>>> Stashed changes
            }

            if (currentGamepad1.b && !previousGamepad1.b)
            {
                arm.setRotation(FRONT);
<<<<<<< Updated upstream
=======
                //tele.addData("rotation power", arm.setRotation(FRONT));
>>>>>>> Stashed changes
            }

            if (gamepad1.dpad_right)
                arm.setExtension(LOW_CHAMBER);
<<<<<<< Updated upstream
            if (gamepad1.dpad_left)
                arm.setExtension(HIGH_CHAMBER);
            if (gamepad1.dpad_down)
                arm.setExtension(FRONTAL_EXTENSION);
            if (gamepad1.dpad_up)
                arm.setExtension(EXTENDED);
=======
                //tele.addData("extension power", arm.setExtension(LOW_CHAMBER));
            if (gamepad1.dpad_left)
                arm.setExtension(HIGH_CHAMBER);
                //tele.addData("extension power", arm.setExtension(HIGH_CHAMBER));
            if (gamepad1.dpad_down)
                arm.setExtension(FRONTAL_EXTENSION);
                //tele.addData("extension power", arm.setExtension(FRONTAL_EXTENSION));
            if (gamepad1.dpad_up)
                arm.setExtension(EXTENDED);
                //tele.addData("extension power", arm.setExtension(EXTENDED));
>>>>>>> Stashed changes


            if (gamepad1.y)
                arm.setExtension(CLOSED);
<<<<<<< Updated upstream
=======
                //tele.addData("extension power", arm.setExtension(CLOSED));
>>>>>>> Stashed changes

            //arm.ROTATION_PIDF.p = p;
            //arm.ROTATION_PIDF.i = i;
            //arm.ROTATION_PIDF.d = d;
            //arm.ROTATION_PIDF.f = f;


            /*
            if (Math.abs(gamepad1.right_stick_y) > 0.01)
                arm.manuallyRotate(-gamepad1.right_stick_y);
            else
                arm.rotationMotor.setPower(0);
            */

            tele.addData("rotation pos", arm.rotationMotor.getCurrentPosition());
            tele.addData("gamepad", gamepad1.right_stick_y);
            tele.addData("extension pos", arm.extensionMotor.getCurrentPosition());
            tele.addData("extension power", -gamepad1.left_stick_y / 10);
            tele.addData("rotation power", arm.pidCalculateRotationPower(arm.targetRotationPos));
            tele.addData("rotation state", arm.rotationState.toString());
            tele.addData("extension state", arm.extensionState.toString());
            tele.addData("rotation angle", arm.getRotationAngle());
            tele.addData("extension length", arm.getExtensionLength());
            tele.addData("hold power", 0.03 * Math.sin(Math.toRadians(arm.rotationAngle)) * arm.extensionLen);
            tele.addData("cog", (arm.m_1 * Math.sqrt(arm.l_1 * arm.l_1 + arm.h_1 * arm.h_1) + arm.m_2 * Math.sqrt(arm.l_2 * arm.l_2 + arm.h_2 * arm.h_2) + arm.m_3 * Math.sqrt(arm.l_3 * arm.l_3 + arm.h_3 * arm.h_3) + arm.m_4 * Math.sqrt(arm.l_4 * arm.l_4 + arm.h_4 * arm.h_4) + arm.m_5 * Math.sqrt(arm.l_5 * arm.l_5 + arm.h_5 * arm.h_5)) / (arm.m_1 + arm.m_2 + arm.m_3 + arm.m_4 + arm.m_5));
            tele.addData("cycle time", timer.milliseconds());
            tele.update();
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
