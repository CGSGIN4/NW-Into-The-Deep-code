package org.firstinspires.ftc.teamcode.robotMovement;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.opencv.core.Mat;

public class drivetrain {
    private DcMotor LeftFrontMotor = null;
    private DcMotor LeftRearMotor = null;
    private DcMotor RightFrontMotor = null;
    private DcMotor RightRearMotor = null;

    public drivetrain(HardwareMap HM){
        LeftFrontMotor = HM.get(DcMotor.class, "MotorLF");
        LeftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftRearMotor = HM.get(DcMotor.class, "MotorLR");
        LeftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightFrontMotor = HM.get(DcMotor.class, "MotorRF");
        RightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightRearMotor = HM.get(DcMotor.class, "MotorRR");
        RightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void applyVector(Vector2d trans, double rotation){
        double x = trans.getX();
        double y = trans.getY();

        double front_left = x + y - rotation;
        double front_right = -x + y + rotation;
        double back_left = -x + y - rotation;
        double back_right = x + y + rotation;

        double max_power = Math.max(Math.abs(front_left), Math.max(Math.abs(front_right), Math.max(Math.abs(back_left), Math.abs(back_right))));
        if (max_power > 1){
            LeftRearMotor.setPower(back_left / max_power);
            LeftFrontMotor.setPower(front_left / max_power);
            RightFrontMotor.setPower(front_right / max_power);
            RightRearMotor.setPower(back_right / max_power);
        }
        else {
            LeftRearMotor.setPower(back_left);
            LeftFrontMotor.setPower(front_left);
            RightFrontMotor.setPower(front_right);
            RightRearMotor.setPower(back_right);
        }
    }

    public void applyVectorFieldCentric(Vector2d trans, double rotation){
        applyVector(trans.rotated(-dataStorage.RobotWorldHeading), rotation);
    }
}
