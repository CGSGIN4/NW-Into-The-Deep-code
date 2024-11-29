package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.FRONTAL_EXTENSION;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.HIGH_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.LOW_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.BACK;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT;
import static java.lang.Math.max;
import static java.lang.Math.min;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.robotMovement.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.utils.GamepadNW;

import java.util.Vector;

@TeleOp
@Config
public class tele_main extends LinearOpMode {
    public static double pitch = 180;
    public static double roll = 0;

    Vector2d gamepad = new Vector2d();
    double turn = 0;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        differential differential = new differential(hardwareMap);
        arm arm = new arm(hardwareMap);

        robot.init();
        dataStorage.init(robot.drive, telemetry, this);

        GamepadNW driverGamepad = new GamepadNW(gamepad1);
        GamepadNW assistGamepad = new GamepadNW(gamepad2);
        int rollPos = 0;
        int pitchPos = 1;
        waitForStart();
        while(opModeIsActive()){
            driverGamepad.update();
            assistGamepad.update();
            dataStorage.updateData();
            arm.update();
            differential.update();

            /* DIFFERENTIAL SECTION */
            if (rollPos > -2 && assistGamepad.isClicked("dpad_left"))
                rollPos--;
            else if (rollPos < 2 && assistGamepad.isClicked("dpad_right"))
                rollPos++;

            if (pitchPos > -1 && assistGamepad.isClicked("dpad_down"))
                pitchPos--;
            else if (pitchPos < 1 && assistGamepad.isClicked("dpad_up"))
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

            //differential.setPitch(pitch);
            //differential.setRoll(roll);

            if (gamepad2.a)
                differential.closeClaw();
            if (gamepad2.b)
                differential.openClaw();

            /* DRIVE SECTION */
            if (Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.left_stick_y) > 0.01 || gamepad1.left_trigger > 0.01 || gamepad1.right_trigger > 0.01)
            {
                gamepad = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                turn = (gamepad1.left_trigger - gamepad1.right_trigger);

                robot.drivetrain.applyVector(gamepad, turn);
            }
            else
                robot.drivetrain.applyVector(new Vector2d(0, 0), 0);


            /* ARM SECTION */
            if (assistGamepad.isClicked("a"))
            {
                if (arm.rotationState == LIFT)
                    arm.setRotation(BACK);
                    //tele.addData("rotation power", arm.setRotation(BACK));
                else if (arm.rotationState == CHAMBER)
                    arm.setRotation(LIFT);
                else
                    arm.setRotation(CHAMBER);
                //tele.addData("rotation power", arm.setRotation(LIFT));
            }

            if (assistGamepad.isClicked("b"))
            {
                arm.setRotation(FRONT);
                //tele.addData("rotation power", arm.setRotation(FRONT));
            }

            if (assistGamepad.isClicked("dpad_right"))
                arm.setExtension(LOW_CHAMBER);
            //tele.addData("extension power", arm.setExtension(LOW_CHAMBER));
            if (assistGamepad.isClicked("dpad_left"))
                arm.setExtension(HIGH_CHAMBER);
            //tele.addData("extension power", arm.setExtension(HIGH_CHAMBER));
            if (assistGamepad.isClicked("dpad_down"))
                arm.setExtension(FRONTAL_EXTENSION);
            //tele.addData("extension power", arm.setExtension(FRONTAL_EXTENSION));
            if (assistGamepad.isClicked("dpad_up"))
                arm.setExtension(EXTENDED);
            //tele.addData("extension power", arm.setExtension(EXTENDED));


            if (assistGamepad.isClicked("y"))
                arm.setExtension(CLOSED);
        }
    }
}
