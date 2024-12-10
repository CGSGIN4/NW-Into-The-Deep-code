package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.FRONTAL_EXTENSION;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.HIGH_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.LOW_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.BACK_HANG1;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.BACK_HANG0;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT;
import static java.lang.Math.decrementExact;
import static java.lang.Math.max;
import static java.lang.Math.min;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    boolean foldingSequence = false;
    boolean unfoldingSequence = false;
    ElapsedTime foldingTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        differential differential = new differential(hardwareMap);
        arm arm = new arm(hardwareMap);
        DcMotor motor1;
        DcMotor motor2;
        motor1 = hardwareMap.get(DcMotor.class, "hangLeft");
        motor2 = hardwareMap.get(DcMotor.class, "hangRight");

        robot.init();
        dataStorage.init(robot.drive, telemetry, this);

        GamepadNW driverGamepad = new GamepadNW(gamepad1);
        GamepadNW assistGamepad = new GamepadNW(gamepad2);
        int rollPos = 0;
        int pitchPos = 1;
        differential.pitchUp();
        differential.rollDefault();
        differential.update();
        waitForStart();
        while(opModeIsActive()) {
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

            dataStorage.telemetry.addData("roll", rollPos);
            dataStorage.telemetry.addData("pitch", pitchPos);
            if (pitchPos > -1 && assistGamepad.isClicked("dpad_down"))
                pitchPos--;
            else if (pitchPos < 1 && assistGamepad.isClicked("dpad_up"))
                pitchPos++;

            switch (rollPos) {
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
                default:
                    break;
            }

            switch (pitchPos) {
                case -1:
                    differential.pitchDown();
                    break;
                case 0:
                    differential.pitchForward();
                    break;
                case 1:
                    differential.pitchUp();
                    break;
                default:
                    break;
            }

            //differential.setPitch(pitch);
            //differential.setRoll(roll);

            if (assistGamepad.isClicked("x"))
                differential.clawSwitch();

            /* DRIVE SECTION */
            if (Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.left_stick_y) > 0.01 || gamepad1.left_trigger > 0.01 || gamepad1.right_trigger > 0.01) {
                gamepad = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                turn = (gamepad1.left_trigger - gamepad1.right_trigger);
                if (driverGamepad.isPressed("y"))
                {
                    turn /= 2;
                }
                robot.drivetrain.applyVector(gamepad, turn);
            } else
                robot.drivetrain.applyVector(new Vector2d(0, 0), 0);


            /* ARM SECTION */
            if (assistGamepad.isClicked("a")) {
                if (arm.rotationState == LIFT)
                    arm.setRotation(FRONT);
                else
                    arm.setRotation(LIFT);
                //tele.addData("rotation power", arm.setRotation(LIFT));
            }

            if (assistGamepad.isClicked("right_bumper")) {
                if (arm.rotationState == LIFT) {
                    if (arm.extensionMotor.getCurrentPosition() < 100) {
                        unfoldingSequence = true;
                    } else {
                        foldingSequence = true;
                        foldingTimer.reset();
                    }
                } else {
                    arm.setExtension(CLOSED);
                    pitchPos = 0;
                }
            }

            if (foldingSequence) {
                differential.openClaw();
                if (foldingTimer.milliseconds() > 270) {
                    differential.pitchDown();
                    if (foldingTimer.milliseconds() > 500) {
                        if (arm.extensionMotor.getCurrentPosition() > 100)
                            arm.setExtension(CLOSED);
                        else if (arm.rotationMotor.getCurrentPosition() > 100)
                            arm.setRotation(FRONT);
                        else {
                            rollPos = 0;
                            pitchPos = 0;
                            foldingSequence = false;
                        }
                    }
                }
            }

            if (unfoldingSequence) {
                if (Math.abs(gamepad2.right_stick_y) > 0.01)
                {
                    unfoldingSequence = false;
                }
                arm.setExtension(EXTENDED);
                if (arm.extensionMotor.getCurrentPosition() > 460) {
                    unfoldingSequence = false;
                }
            }


            if (Math.abs(gamepad2.right_stick_y) > 0.01)
                arm.manuallyExtend(-gamepad2.right_stick_y);
            else {
                if (arm.extensionState == MANUAL)
                    arm.extensionMotor.setPower(0);
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.1)
            {
                motor1.setPower(gamepad2.left_stick_y);
                motor2.setPower(-gamepad2.left_stick_y);
            }
            else
            {
                motor1.setPower(0);
                motor2.setPower(0);
            }
            if (gamepad2.left_trigger > 0.1)
                arm.setRotation(BACK_HANG1);
            else if (gamepad2.right_trigger > 0.1)
                arm.setRotation(BACK_HANG0);
        }
    }
}
