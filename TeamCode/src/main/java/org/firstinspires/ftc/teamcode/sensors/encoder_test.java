package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class encoder_test extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode(){
        DcMotor enc = hardwareMap.get(DcMotor.class, "lift_motor_right");
        int start = enc.getCurrentPosition();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("ticks", enc.getCurrentPosition() - start);
            telemetry.update();
        }
    }
}
