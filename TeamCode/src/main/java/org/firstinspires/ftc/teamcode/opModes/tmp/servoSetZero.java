package org.firstinspires.ftc.teamcode.opModes.tmp;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class servoSetZero extends LinearOpMode {
    public static double pos = 0.7;
    NormalizedColorSensor colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo;
        servo = hardwareMap.get(Servo.class, "servo");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(2);
        float[] hsvValues = new float[3];
        waitForStart();
        MultipleTelemetry tele = new MultipleTelemetry(telemetry);
        while(opModeIsActive()){
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (gamepad1.a)
            {
                if (colorSensor instanceof SwitchableLight) {
                    SwitchableLight light = (SwitchableLight)colorSensor;
                    light.enableLight(!light.isLightOn());
                }
            }
            servo.setPosition(pos);

            tele.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            tele.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            tele.addData("Alpha", "%.3f", colors.alpha);
            if (colorSensor instanceof DistanceSensor) {
                tele.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }
            tele.update();
        }
    }
}
