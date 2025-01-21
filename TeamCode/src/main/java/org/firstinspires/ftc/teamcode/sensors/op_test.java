package org.firstinspires.ftc.teamcode.sensors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.calculator;
import org.firstinspires.ftc.teamcode.subsystems.modules.sonar_localizer;
import org.firstinspires.ftc.teamcode.utils.painter;

//@Disabled
@TeleOp
public class op_test extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime sensorTimer = new ElapsedTime();
    public FtcDashboard dashboard;
    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        painter painter = new painter();
        drive.setPoseEstimate(new Pose2d(48, 48, -Math.PI / 2));
        //MB1242 forward_sensor = hardwareMap.get(MB1242.class, "forwardSensor");
        //MB1643 left_sensor = hardwareMap.get(MB1643.class, "leftSensor");
        AnalogInput fS = hardwareMap.get(AnalogInput.class, "frontSensor");
        AnalogInput lS = hardwareMap.get(AnalogInput.class, "leftSensor");
        AnalogInput riS = hardwareMap.get(AnalogInput.class, "rightSensor");
        AnalogInput reS = hardwareMap.get(AnalogInput.class, "rearSensor");
        sonar frontSensor = new sonar(fS, new Vector2d(0, 0), 0, sonar.sonar_type.TWO_EYED);
        sonar leftSensor = new sonar(lS, new Vector2d(4.72, 5.71), Math.PI / 2, sonar.sonar_type.TWO_EYED);
        sonar rightSensor = new sonar(riS, new Vector2d(0, 0), -Math.PI / 2, sonar.sonar_type.TWO_EYED);
        sonar rearSensor = new sonar(reS, new Vector2d(-4.72, -4.13), Math.PI, sonar.sonar_type.TWO_EYED);
        sonar_localizer sonarLocalizer = new sonar_localizer(frontSensor, leftSensor, rearSensor, rightSensor);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        dataStorage.init(drive, telemetry, this);
        waitForStart();
        sensorTimer.reset();
        //dataStorage.init(new SampleMecanumDrive(hardwareMap), telemetry, this);
        while (opModeIsActive()) {
            timer.reset();
            dataStorage.updateData();
            sonarLocalizer.ping();
            while (timer.milliseconds() < 250);
            TelemetryPacket packet = new TelemetryPacket(true);
            painter.prepare(packet, packet.fieldOverlay());

            painter.drawRobot(dataStorage.RobotWorldX, dataStorage.RobotWorldY, dataStorage.RobotWorldHeading, "pink");
            //drive.setPoseEstimate(new Pose2d(sonarLocalizer.getPosition(), dataStorage.RobotWorldHeading));
            //painter.drawPoint(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());
            painter.drawPoint(rearSensor.OFFSET.rotated(dataStorage.RobotWorldHeading).plus(dataStorage.RobotPose).getX(), rearSensor.OFFSET.rotated(dataStorage.RobotWorldHeading).plus(dataStorage.RobotPose).getY(), "red");
            painter.drawPoint(leftSensor.OFFSET.rotated(dataStorage.RobotWorldHeading).plus(dataStorage.RobotPose).getX(), leftSensor.OFFSET.rotated(dataStorage.RobotWorldHeading).plus(dataStorage.RobotPose).getY(), "green");
            painter.drawVector(new Vector2d(0, -100), new Vector2d(0, 50), "blue");
            painter.drawVector(new Vector2d(-100, 0), new Vector2d(50, 0), "purple");
            painter.drawVector(dataStorage.RobotPose.plus(rearSensor.OFFSET.rotated(dataStorage.RobotWorldHeading)), dataStorage.RobotPose.plus(rearSensor.OFFSET.rotated(dataStorage.RobotWorldHeading)).plus(new Vector2d(300, 0).rotated(dataStorage.RobotWorldHeading + rearSensor.ANGULAR_OFFSET)), "red");
            painter.drawVector(dataStorage.RobotPose.plus(leftSensor.OFFSET.rotated(dataStorage.RobotWorldHeading)), dataStorage.RobotPose.plus(leftSensor.OFFSET.rotated(dataStorage.RobotWorldHeading)).plus(new Vector2d(300, 0).rotated(dataStorage.RobotWorldHeading + leftSensor.ANGULAR_OFFSET)), "green");
            painter.drawPoint(sonarLocalizer.getPosition().getX(), sonarLocalizer.getPosition().getY());
            painter.drawVector(new Vector2d(72, -72), new Vector2d(-72, -72), "red");
            painter.drawVector(new Vector2d(72, -72), new Vector2d(72, 72), "black");
            painter.drawVector(new Vector2d(-72, 72), new Vector2d(72, 72), "blue");
            painter.drawVector(new Vector2d(-72, 72), new Vector2d(-72, -72), "green");
            sensorTimer.reset();

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
