package org.firstinspires.ftc.teamcode.opModes.tmp;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;

@TeleOp
@Config
public class differentialTest extends LinearOpMode {
    public static double pitch;
    public static double roll;

    @Override
    public void runOpMode() throws InterruptedException {
        differential differential = new differential(hardwareMap);
        waitForStart();
        MultipleTelemetry tele = new MultipleTelemetry(telemetry);
        while(opModeIsActive()){
            differential.setPitch(pitch);
            differential.setRoll(roll);
            differential.update();

            tele.addData("lTheta", differential.lTheta);
            tele.addData("rTheta", differential.rTheta);
            tele.addData("pitch", differential.pitch);
            tele.addData("roll", differential.roll);
            tele.update();
        }
    }
}
