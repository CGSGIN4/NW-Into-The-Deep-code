package org.firstinspires.ftc.teamcode.opModes.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;
@Disabled
@TeleOp(group = "Deprecated")
public class TakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor1;
        DcMotor motor2;
        motor1 = hardwareMap.get(DcMotor.class, "hangLeft");
        motor2 = hardwareMap.get(DcMotor.class, "hangRight");
        waitForStart();
        while(opModeIsActive()){
            motor1.setPower(gamepad1.left_stick_y);
            motor2.setPower(-gamepad1.left_stick_y);
        }
    }
}
