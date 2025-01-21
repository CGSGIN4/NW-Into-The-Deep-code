package org.firstinspires.ftc.teamcode.opModes.tests;

import static java.lang.Math.max;
import static java.lang.Math.min;

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
import org.firstinspires.ftc.teamcode.utils.GamepadNW;

@TeleOp(group = "Tests")
@Config
public class differentialTest extends LinearOpMode {
    public static double pitch = 180;
    public static double roll = 0;
    public static double clawPos = 0.32;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        differential differential = new differential(hardwareMap);
        GamepadNW driverGamepad = new GamepadNW(gamepad1);
        GamepadNW assistGamepad = new GamepadNW(gamepad2);
        claw = hardwareMap.get(Servo.class, "claw");
        int rollPos = 0;
        int pitchPos = 1;
        waitForStart();
        MultipleTelemetry tele = new MultipleTelemetry(telemetry);
        while(opModeIsActive()){
            driverGamepad.update();
            assistGamepad.update();

            pitch += gamepad1.left_stick_y * 10;
            pitch = max(0, min(180, pitch));
            roll += gamepad1.left_stick_x * 10;
            roll = max(-80, min(80, roll));

            /*
            if (rollPos > -2 && driverGamepad.isClicked("dpad_left"))
                rollPos--;
            else if (rollPos < 2 && driverGamepad.isClicked("dpad_right"))
                rollPos++;

            if (pitchPos > -1 && driverGamepad.isClicked("dpad_down"))
                pitchPos--;
            else if (pitchPos < 1 && driverGamepad.isClicked("dpad_up"))
                pitchPos++;

            switch (rollPos){
                case -2:
                    differential.rollFullLeft();
                    break;
                case -1:
                    differential.rollHalfLeft();
                    break;
                case 0:
                    differential.rollDefault();
                    break;
                case 1:
                    differential.rollHalfRight();
                    break;
                case 2:
                    differential.rollFullRight();
                    break;
            }

            switch (pitchPos){
                case -1:
                    differential.pitchDown();
                    break;
                case 0:
                    differential.pitchForward();
                    break;
                case 1:
                    differential.pitchUp();
                    break;
            }

             */

            differential.setPitch(pitch);
            differential.setRoll(roll);
            differential.update();

            /*
            if (gamepad1.a)
                differential.closeClaw();
            if (gamepad1.b)
                differential.openClaw();
            */
            claw.setPosition(clawPos);
            tele.addData("lTheta", differential.lTheta);
            tele.addData("rTheta", differential.rTheta);
            tele.addData("pitch", differential.pitch);
            tele.addData("roll", differential.roll);
            tele.update();
        }
    }
}
