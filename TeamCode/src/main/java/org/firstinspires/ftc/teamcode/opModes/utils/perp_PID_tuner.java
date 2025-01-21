package org.firstinspires.ftc.teamcode.opModes.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.velocity_calculator;

@Autonomous(group = "Utils")
public class perp_PID_tuner extends LinearOpMode {

    Robot robot;
    path_follower path_follower;
    ElapsedTime timer = new ElapsedTime();
    Vector2d[] nodes = new Vector2d[4];
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.init();
        dataStorage.init(robot.drive, telemetry, this);
        path_follower = new path_follower(robot.drivetrain);
        robot.drive.setPoseEstimate(new Pose2d(0, 48, Math.PI / 2));
        velocity_calculator velocity_calculator = new velocity_calculator();

        waitForStart();

        dataStorage.updateData();
        while (dataStorage.RobotPose.minus(new Vector2d(-48, -48)).norm() > 1 || dataStorage.RobotVelocity.norm() > 3) {
            dataStorage.updateData();
            Pose2d pid = velocity_calculator.getPIDpower(new Vector2d(-48, -48), -Math.PI / 2);
            robot.drivetrain.applyVectorFieldCentric(pid.vec().rotated(Math.toRadians(90)), pid.getHeading());
        }

        robot.stop();
        timer.reset();
        while(timer.seconds() < 2 && opModeIsActive());
    }
}
