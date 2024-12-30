package org.firstinspires.ftc.teamcode.opModes.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Deprecated")
public class TakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor1;
        motor1 = hardwareMap.get(DcMotorEx.class, "armExtensionMotor");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("voltage", motor1.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            if (gamepad1.a)
                motor1.setPower(1);
            else if (gamepad1.b)
                motor1.setPower(-1);
            else
                motor1.setPower(0);
        }
    }
}
