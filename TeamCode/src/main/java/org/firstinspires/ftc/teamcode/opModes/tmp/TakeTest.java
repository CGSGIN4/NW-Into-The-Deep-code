package org.firstinspires.ftc.teamcode.opModes.tmp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp
public class TakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor servo;
        servo = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        while(opModeIsActive()){
            servo.setPower(0.5);
        }
    }
}
