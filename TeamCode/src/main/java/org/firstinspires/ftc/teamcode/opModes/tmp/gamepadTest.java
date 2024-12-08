package org.firstinspires.ftc.teamcode.opModes.tmp;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.FRONTAL_EXTENSION;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.HIGH_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.LOW_CHAMBER;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.velocity_calculator;
import org.firstinspires.ftc.teamcode.utils.GamepadNW;
import org.firstinspires.ftc.teamcode.utils.painter;

@TeleOp
public class gamepadTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        MultipleTelemetry tele = new MultipleTelemetry(dashboardTelemetry, telemetry);
        double time = 0;
        GamepadNW driverGamepad = new GamepadNW(gamepad1);
        GamepadNW assistGamepad = new GamepadNW(gamepad2);
        double cnt = 0;
        DcMotor motor = hardwareMap.get(DcMotor.class, "armExtensionMotor");

        waitForStart();

        while (opModeIsActive()) {
            timer.reset();
            driverGamepad.update();
            assistGamepad.update();
            time = timer.milliseconds();

            telemetry.addData("time", time);
            if (driverGamepad.isClicked("a")){
                cnt++;
            }
            telemetry.addData("cnt", cnt);
            if (driverGamepad.isPressed("a")){
                telemetry.addData("ap", "pressed");
                motor.setPower(0.004);
            }
            else {
                telemetry.addData("ap", "not pressed");
                motor.setPower(0);
            }
            telemetry.update();
        }
    }
}
