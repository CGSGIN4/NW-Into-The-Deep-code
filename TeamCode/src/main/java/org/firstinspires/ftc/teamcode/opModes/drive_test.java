package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.velocity_calculator;
import org.firstinspires.ftc.teamcode.utils.painter;

@TeleOp
public class drive_test extends LinearOpMode {
    Robot robot;
    path_follower path_follower;

    Vector2d gamepad;
    double last_turn = 0;
    double turn = 0;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        path_follower = new path_follower(robot.drivetrain);
        velocity_calculator velocity_calculator = new velocity_calculator();
        robot.init();
        dataStorage.init(robot.drive, telemetry, this);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        double targetHeading = 0;
        ElapsedTime angleTimer = new ElapsedTime();

        double maxVel = 0;
        waitForStart();
        painter painter = new painter();

        robot.drive.setPoseEstimate(new Pose2d(0, 0, 0));
        while (opModeIsActive()) {
            dataStorage.updateData();

            if (Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.left_stick_y) > 0.01 || gamepad1.left_trigger > 0.01 || gamepad1.right_trigger > 0.01)
            {
                gamepad = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                turn = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.8;

                robot.drivetrain.applyVector(gamepad, turn);
            }
            else
                robot.drivetrain.applyVector(new Vector2d(0, 0), 0);

            last_turn = turn;

            if (gamepad1.a)
            {
                dataStorage.poseHistory.clear();
            }

            if (gamepad1.b)
            {
                dataStorage.lastPoses.dump();
            }

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            painter.prepare(packet, fieldOverlay);
            //painter.drawPolyLine(dataStorage.poseHistory.toArray(new Vector2d[0]));
            painter.drawRobot(dataStorage.RobotWorldX, dataStorage.RobotWorldY, dataStorage.RobotWorldHeading, "pink");
            packet.put("heading", dataStorage.RobotWorldHeading);
            packet.put("x", dataStorage.RobotWorldX);
            packet.put("y", dataStorage.RobotWorldY);
            dashboard.sendTelemetryPacket(packet);

            /*
            maxVel = Math.max(maxVel, dataStorage.RobotVelocity.norm());
            telemetry.addData("velocity", dataStorage.RobotVelocity.norm());
            telemetry.addData("max vel", maxVel);
            telemetry.update();
            */
        }
    }
}
