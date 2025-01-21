package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.robotMovement.drivetrain;

public class Robot implements org.firstinspires.ftc.teamcode.classTemplates.Robot {

    HardwareMap hardwareMap;
    public SampleMecanumDrive drive;
    public drivetrain drivetrain;

    public Robot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void init() {
        this.drivetrain = new drivetrain(hardwareMap);
    }

    @Override
    public void stop() {
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }
}
