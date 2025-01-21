package org.firstinspires.ftc.teamcode.opModes.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.calculator;
import org.firstinspires.ftc.teamcode.utils.painter;

@Autonomous(group = "Utils")
public class breakAccel_tuner extends LinearOpMode {
    Robot robot;
    ElapsedTime timer = new ElapsedTime();
    int cnt = 0;
    double accel;
    double sumAccel;
    double sumVel;
    double startVelocity;
    Vector2d start;
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.init();
        cnt = 0;
        sumAccel = 0.;
        sumVel = 0.;
        dataStorage.init(robot.drive, telemetry, this);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        painter painter = new painter();

        waitForStart();

        while (cnt < 16 && opModeIsActive()){
            dataStorage.updateData();
            start = dataStorage.RobotPose;
            robot.drivetrain.applyVector(new Vector2d(0, 1.0 * Math.signum(cnt % 2 - 0.5)), 0);
            while (start.distTo(dataStorage.RobotPose) < 72 && opModeIsActive()) {
                dataStorage.updateData();
                TelemetryPacket packet = new TelemetryPacket(true);
                painter.prepare(packet, packet.fieldOverlay());
                painter.drawRobot(dataStorage.RobotWorldX, dataStorage.RobotWorldY, dataStorage.RobotWorldHeading, "pink");
                dashboard.sendTelemetryPacket(packet);
            }
            robot.drivetrain.applyVector(new Vector2d(0, 0), 0);
            timer.reset();
            startVelocity = dataStorage.RobotVelocity.norm();
            while (dataStorage.RobotVelocity.norm() > 0.1 && opModeIsActive()) {
                dataStorage.updateData();
                TelemetryPacket packet = new TelemetryPacket(true);
                painter.prepare(packet, packet.fieldOverlay());
                painter.drawRobot(dataStorage.RobotWorldX, dataStorage.RobotWorldY, dataStorage.RobotWorldHeading, "pink");
                dashboard.sendTelemetryPacket(packet);
            }
            accel = Math.abs(startVelocity - dataStorage.RobotVelocity.norm()) / timer.seconds();
            sumAccel += accel;
            sumVel += startVelocity;
            telemetry.addData("accel", accel);
            cnt++;
            telemetry.addData("average accel", sumAccel / cnt);
            telemetry.addData("average start vel", sumVel / cnt);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(true);
            painter.prepare(packet, packet.fieldOverlay());
            painter.drawRobot(dataStorage.RobotWorldX, dataStorage.RobotWorldY, dataStorage.RobotWorldHeading, "pink");
            packet.put("cnt", cnt);
            packet.put("accel", accel);
            packet.put("average accel", sumAccel / cnt);
            packet.put("average start vel", sumVel / cnt);

            dashboard.sendTelemetryPacket(packet);
        }
        robot.drivetrain.applyVector(new Vector2d(0, 0), 0);
        telemetry.addData("average accel", sumAccel / cnt);
        telemetry.addData("average start vel", sumVel / cnt);
        telemetry.update();
        timer.reset();
        while(timer.seconds() < 30 && opModeIsActive());
    }
}
