package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.LOW_BASKET;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.BACK_HANG1;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.RESET;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.modules.hang;
import org.firstinspires.ftc.teamcode.utils.GamepadNW;

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
    boolean unfoldingSequenceLowBasket = false;
    boolean intakingSequence = false;
    ElapsedTime foldingTimer = new ElapsedTime();
    ElapsedTime intakingTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        differential differential = new differential(hardwareMap);
        arm arm = new arm(hardwareMap);
        hang hang = new hang(hardwareMap);

        robot.init();
        dataStorage.init(robot.drive, telemetry, this);

        GamepadNW driverGamepad = new GamepadNW(gamepad1);
        GamepadNW assistGamepad = new GamepadNW(gamepad2);

        waitForStart();
        differential.closeClaw();
        while(opModeIsActive()) {
            driverGamepad.update();
            assistGamepad.update();
            dataStorage.updateData();
            arm.update();
            hang.update(dataStorage.telemetry);
            differential.update();

            /* DIFFERENTIAL SECTION */
            if (assistGamepad.isClicked("dpad_down"))
                if (pitch > 100)
                    pitch = 100;
                else
                    pitch = 20;
            if (assistGamepad.isClicked("dpad_up"))
                if (pitch < 100)
                    pitch = 100;
                else
                    pitch = 180;

            if (assistGamepad.isClicked("dpad_left"))
                if (roll >= -80 && roll <= -45)
                    roll = -80;
                else if (roll > -45 && roll <= 0)
                    roll = -45;
                else if (roll > 0 && roll <= 45)
                    roll = 0;
                else
                    roll = 45;
            if (assistGamepad.isClicked("dpad_right"))
                if (roll <= 80 && roll >= 45)
                    roll = 80;
                else if (roll >= 0 && roll < 45)
                    roll = 45;
                else if (roll >= -45 && roll < 0)
                    roll = 0;
                else
                    roll = -45;

            if (gamepad2.left_trigger > 0.05 && roll > -80)
                roll -= gamepad2.left_trigger * 4;
            if (gamepad2.right_trigger > 0.05 && roll < 80)
                roll += gamepad2.right_trigger * 4;

            differential.setRoll(roll);
            differential.setPitch(pitch);

            if (assistGamepad.isClicked("x"))
                differential.clawSwitch();

            /* DRIVE SECTION */
            if (Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.left_stick_y) > 0.01 || gamepad1.left_trigger > 0.01 || gamepad1.right_trigger > 0.01) {
                gamepad = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                turn = (gamepad1.left_trigger - gamepad1.right_trigger);
                if (Math.abs(gamepad1.right_stick_y) > 0.05 || Math.abs(gamepad1.right_stick_x) > 0.05)
                    robot.drivetrain.applyVector(gamepad.div(2), turn / 2);
                else
                    robot.drivetrain.applyVector(gamepad, turn);
            } else
                robot.drivetrain.applyVector(new Vector2d(0, 0), 0);


            /* ARM SECTION */
            if (assistGamepad.isClicked("a")) {
                if (arm.rotationState == LIFT)
                    arm.setRotation(FRONT);
                else {
                    arm.setRotation(LIFT);
                    pitch = 20;
                    roll = 0;
                }
                //tele.addData("rotation power", arm.setRotation(LIFT));
            }
            if (assistGamepad.isClicked("right_bumper")) {
                if (arm.rotationState == LIFT) {
                    if (arm.extensionMotor.getCurrentPosition() < 100) {
                        unfoldingSequenceLowBasket = false;
                        unfoldingSequence = true;
                    } else {
                        pitch = 145;
                        foldingSequence = true;
                        foldingTimer.reset();
                    }
                } else {
                    intakingSequence = true;
                    intakingTimer.reset();
                }
            }

            if (assistGamepad.isClicked("left_bumper") && arm.rotationState == LIFT) {
                unfoldingSequenceLowBasket = true;
                unfoldingSequence = false;
            }

            /* TELEOP AUTOMATIONS SECTION */
            if (foldingSequence) {
                unfoldingSequenceLowBasket = false;
                unfoldingSequence = false;
                if (foldingTimer.milliseconds() > 300 && foldingTimer.milliseconds() < 500) /* perekid */
                    differential.openClaw();
                else if (foldingTimer.milliseconds() > 500 && foldingTimer.milliseconds() < 800)
                    pitch = 20;
                else
                if (foldingTimer.milliseconds() > 800) {
                    if (arm.extensionMotor.getCurrentPosition() > 100)
                        arm.setExtension(CLOSED);
                    else if (arm.rotationMotor.getCurrentPosition() > 100)
                        arm.setRotation(FRONT);
                    else {
                        roll = 0;
                        pitch = 100;
                        foldingSequence = false;
                    }
                }
            }

            if (unfoldingSequence) {
                unfoldingSequenceLowBasket = false;
                if (Math.abs(gamepad2.right_stick_y) > 0.01)
                    unfoldingSequence = false;

                pitch = 20;
                arm.setExtension(EXTENDED);
                if (arm.extensionMotor.getCurrentPosition() > 460) {
                    unfoldingSequence = false;
                }
            }

            if (unfoldingSequenceLowBasket) {
                unfoldingSequence = false;
                if (Math.abs(gamepad2.right_stick_y) > 0.01)
                    unfoldingSequenceLowBasket = false;

                pitch = 20;
                arm.setExtension(LOW_BASKET);
                if (arm.extensionMotor.getCurrentPosition() > 180)
                    unfoldingSequenceLowBasket = false;
            }

            if (intakingSequence) {
                if (intakingTimer.milliseconds() < 200)
                    differential.closeClaw();
                else {
                    arm.setExtension(CLOSED);
                    pitch = 180;
                    intakingSequence = false;
                }
            }

            if (Math.abs(gamepad2.right_stick_y) > 0.01) {
                arm.manuallyExtend(-gamepad2.right_stick_y);
                if (gamepad2.right_stick_y < -0.1 && arm.rotationState == RESET)
                    differential.openClaw();
            }
            else {
                if (arm.extensionState == MANUAL)
                    arm.extensionMotor.setPower(0);
            }

            if (assistGamepad.isClicked("y")) {
                arm.setRotation(BACK_HANG1);
                arm.setExtension(CLOSED);
                differential.openClaw();
                differential.pitchUp();
                if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING)
                    hang.prepare();
                else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.ASCEND2COMPLETE)
                    hang.prepare3();
                else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.PREPARE3)
                    hang.standby3();
                else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.STANDBY3)
                    hang.fold3();
            }
            if (assistGamepad.isClicked("b")) {
                hang.chill();
                hang.state = org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING;
            }

            if (assistGamepad.isClicked("start"))
                arm.resetExtensionEncoders();
            if (assistGamepad.isClicked("back"))
                arm.resetRotationEncoders();
            if (assistGamepad.isPressed("left_stick_button"))
                arm.manuallyRotate(-0.5);
        }
    }
}
