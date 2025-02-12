package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.LOW_BASKET;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.SUPPORT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.RESET;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.data.transfer;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.modules.hang;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.velocity_calculator;
import org.firstinspires.ftc.teamcode.utils.GamepadNW;
import org.firstinspires.ftc.teamcode.utils.logger;

import java.io.IOException;

@TeleOp
@Config
public class tele_main extends LinearOpMode {
    double TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING = 0;
    int scoring_mode = 0;
    double scored = 0;
    public static double pitch = 6;
    public static double roll = -11;
    Vector2d gamepad = new Vector2d();
    double turn = 0;
    FtcDashboard dashboard;
    boolean foldingSequence = false;
    boolean unfoldingSequence = false;
    boolean unfoldingSequenceLowBasket = false;
    boolean intakingSequence = false;
    boolean fast_pitch_swap = false;
    boolean hang_called = false;
    String lastCall;
    ElapsedTime foldingTimer = new ElapsedTime();
    ElapsedTime intakingTimer = new ElapsedTime();
    ElapsedTime hangTimer = new ElapsedTime();
    path_follower path_follower;

    @Override
    public void runOpMode() throws InterruptedException {
        //logger.init();
        Robot robot = new Robot(hardwareMap);
        path_follower = new path_follower(robot.drivetrain);
        velocity_calculator velocity_calculator = new velocity_calculator();

        differential differential = new differential(hardwareMap);
        arm arm = new arm(hardwareMap, true);
        hang hang = new hang(hardwareMap);

        robot.init();
        dataStorage.init(robot.drive, telemetry, this);
        dataStorage.opModeIsAutonomous = false;

        GamepadNW driverGamepad = new GamepadNW(gamepad1);
        GamepadNW assistGamepad = new GamepadNW(gamepad2);

        waitForStart();
        //logger.writeLn("started teleop");
        robot.drive.setPoseEstimate(new Pose2d(dataStorage.RobotWorldX, dataStorage.RobotWorldY, transfer.angle));
        differential.openClaw();
        while(opModeIsActive()) {
            driverGamepad.update();
            assistGamepad.update();
            dataStorage.updateData();
            arm.update(dataStorage.telemetry);
            hang.update(dataStorage.telemetry);
            differential.update(dataStorage.telemetry);

            /* DIFFERENTIAL SECTION */
            if (assistGamepad.isClicked("dpad_down")) {
                lastCall = "dpad";
                if (fast_pitch_swap) {
                    pitch = 6;
                    fast_pitch_swap = false;
                }
                if (pitch > 99)
                    pitch = 99;
                else
                    pitch = 6;
                if (arm.rotationState == RESET)
                    differential.openClaw();
            }
            if (assistGamepad.isClicked("dpad_up")) {
                if (pitch < 99)
                    pitch = 99;
                else
                    pitch = 176;
                lastCall = "dpad_up";
            }
            if (assistGamepad.isClicked("dpad_left")) {
                if (roll >= -80 && roll <= -45)
                    roll = -80;
                else if (roll > -45 && roll <= -11)
                    roll = -45;
                else if (roll > -11 && roll <= 45)
                    roll = -11;
                else
                    roll = 45;
            }
            if (assistGamepad.isClicked("dpad_right")) {
                if (roll <= 75 && roll >= 45)
                    roll = 75;
                else if (roll >= -11 && roll < 45)
                    roll = 45;
                else if (roll >= -45 && roll < -11)
                    roll = -11;
                else
                    roll = -45;
            }

            differential.setRoll(roll);
            differential.setPitch(pitch);

            if (assistGamepad.isClicked("x"))
                differential.clawSwitch();

            if (driverGamepad.isClicked("start"))
                robot.drive.setPoseEstimate(new Pose2d(dataStorage.RobotWorldX, dataStorage.RobotWorldY, -Math.PI));

            /* DRIVE SECTION */
            if (Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.left_stick_y) > 0.01 || gamepad1.left_trigger > 0.01 || gamepad1.right_trigger > 0.01) {
                gamepad = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y);

                if (driverGamepad.isPressed("right_bumper"))
                    turn = path_follower.velocity_calculator.getRotationCustomDirection(-Math.PI);
                else if (driverGamepad.isPressed("left_bumper"))
                    turn = path_follower.velocity_calculator.getRotationCustomDirection(-Math.PI * 3 / 4);
                else
                    turn = (gamepad1.left_trigger - gamepad1.right_trigger);

                if (arm.extensionMotor.getCurrentPosition() > arm.EXTENSION_FRONT_MAX && arm.rotationState == LIFT)
                    turn *= 0.4;

                if (Math.abs(gamepad1.right_stick_y) > 0.05 || Math.abs(gamepad1.right_stick_x) > 0.05)
                    robot.drivetrain.applyVector(gamepad.div(2), turn / 2);
                else
                    robot.drivetrain.applyVector(gamepad, turn);
            } else
                robot.drivetrain.applyVector(new Vector2d(0, 0), 0);


            /* ARM SECTION */
            if (assistGamepad.isClicked("a")) {
                if (arm.rotationState == LIFT) {
                    if (arm.extensionMotor.getCurrentPosition() + arm.offset < 435) {
                        arm.setRotation(FRONT);
                        arm.setExtension(CLOSED);
                        pitch = 176;
                        lastCall = "gamepad a";
                    }
                }
                else {
                    arm.setRotation(LIFT);
                    arm.setExtension(SUPPORT);
                    lastCall = ("called by pressing X in rotation RESET");
                    pitch = 6;
                    roll = -11;
                }
                //tele.addData("rotation power", arm.setRotation(LIFT));
            }
            if (assistGamepad.isClicked("right_bumper")) {
                if (arm.rotationState == LIFT) {
                    if (arm.extensionMotor.getCurrentPosition() + arm.offset < 280) {
                        unfoldingSequenceLowBasket = false;
                        unfoldingSequence = true;
                        //logger.writeLn("activated unfolding sequence");
                    } else {
                        if (pitch == 6)
                            TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING = 500;
                        pitch = 125;
                        foldingSequence = true;
                        foldingTimer.reset();
                        //logger.writeLn("activated folding sequence");
                    }
                } else {
                    intakingSequence = true;
                    intakingTimer.reset();
                    //logger.writeLn("activated intaking sequence");
                }
            }

            if (assistGamepad.isClicked("left_bumper") && arm.rotationState == LIFT) {
                unfoldingSequenceLowBasket = true;
                unfoldingSequence = false;
                //logger.writeLn("activated unfoldingLowBasket sequence");
            }

            if (assistGamepad.isClicked("b")) {
                scoring_mode = (scoring_mode == 1 ? 0 : 1);
                //logger.writeLn("scoring mode set to " + scoring_mode);
                if (scoring_mode == 1)
                    TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING = 500;
                else
                    TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING = 0;
            }

            /* TELEOP AUTOMATIONS SECTION */
            if (foldingSequence) {
                unfoldingSequenceLowBasket = false;
                unfoldingSequence = false;
                intakingSequence = false;
                if (foldingTimer.milliseconds() < TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 500 && arm.extensionState != MANUAL && arm.extensionMotor.getCurrentPosition() > arm.EXTENSION_FRONT_MAX)
                    arm.setExtension(EXTENDED);
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING) /* perekid */
                    differential.openClaw();
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 200) {
                    pitch = 6;
                    lastCall = "folding seq";
                }
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 500) {
                    if (arm.extensionMotor.getCurrentPosition() + arm.offset > 100) {
                        arm.setExtension(CLOSED);
                    }
                    else {
                        arm.setRotation(FRONT);
                        roll = -11;
                        foldingSequence = false;
                        pitch = 176;
                        fast_pitch_swap = true;
                        //logger.writeLn("folding sequence finished as planned");
                        lastCall = "folding end";
                    }
                }
            }

            if (unfoldingSequence) {
                unfoldingSequenceLowBasket = false;
                if (Math.abs(gamepad2.right_stick_y) > 0.01)
                    unfoldingSequence = false;

                if (scoring_mode == 1) {
                    pitch = 6;
                    lastCall = "unfolding seq";
                }
                else
                    pitch = 125;
                arm.setExtension(EXTENDED);
                if (arm.extensionMotor.getCurrentPosition() + arm.offset > 1320) {
                    unfoldingSequence = false;
                    //logger.writeLn("unfolding sequence finished as planned");
                }
            }

            if (unfoldingSequenceLowBasket) {
                unfoldingSequence = false;
                if (Math.abs(gamepad2.right_stick_y) > 0.01)
                    unfoldingSequenceLowBasket = false;

                if (scoring_mode == 1)
                    pitch = 6;
                else
                    pitch = 125;
                arm.setExtension(LOW_BASKET);
                if (arm.extensionMotor.getCurrentPosition() + arm.offset > 465) {
                    unfoldingSequenceLowBasket = false;
                    //logger.writeLn("unfolding sequenceLowBasket finished as planned");
                }
            }

            if (intakingSequence) {
                if (intakingTimer.milliseconds() < 200)
                    differential.closeClaw();
                else {
                    arm.setExtension(CLOSED);
                    pitch = 130;
                    intakingSequence = false;
                    //logger.writeLn("intaking sequence finished as planned");
                }
            }

            if (Math.abs(gamepad2.right_stick_y) > 0.01) {
                //logger.writeLn("gamepad2.right_stick_y triggered");
                arm.manuallyExtend(-gamepad2.right_stick_y);
                //if (gamepad2.right_stick_y < -0.1 && arm.rotationState == RESET)
                    //differential.openClaw();
            }
            else {
                if (arm.extensionState == MANUAL) {
                    arm.extensionMotor.setPower(0);
                }
            }

            if (arm.rotationState == RESET) {
                if (gamepad2.right_trigger > 0.05 && arm.extensionMotor.getCurrentPosition() + arm.offset < 305) {
                    arm.pidExtend(arm.targetExtensionPos + 7);
                }
                if (gamepad2.left_trigger > 0.05) {
                    arm.pidExtend(arm.targetExtensionPos - 7);
                }
            }

            if (assistGamepad.isClicked("y")) {
                unfoldingSequence = false;
                unfoldingSequenceLowBasket = false;
                intakingSequence = false;
                foldingSequence = false;
                hang_called = true;
                if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING && pitch != 176) {
                    hangTimer.reset();
                    pitch = 176;
                    roll = -11;
                    lastCall = "hang";
                }
                arm.setExtension(CLOSED);
                differential.openClaw();
                if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.ASCEND2COMPLETE || hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.FOLDING)
                    hang.prepare3();
                else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.PREPARE3)
                    hang.standby3();
                else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.STANDBY3)
                    hang.fold3();
            }
            if (assistGamepad.isClicked("b") && hang.state != org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING) {
                hang.chill();
                hang.state = org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING;
                hang_called = false;
            }

            if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING && hangTimer.milliseconds() > 300 && hang_called)
            {
                arm.setRotation(LIFT);
                hang.prepare();
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                hang.setPower(-gamepad2.left_stick_y);
                hang.state = org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING;
                hang_called = false;
            }
            else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING)
                hang.setPower(0);

            if (assistGamepad.isClicked("start"))
                arm.resetExtensionEncoders();
            if (assistGamepad.isClicked("back"))
                arm.resetRotationEncoders();
            if (assistGamepad.isPressed("left_stick_button"))
                arm.manuallyRotate(-0.5);

            dataStorage.telemetry.addLine("-----------TELEOP----------");
            dataStorage.telemetry.addLine(lastCall);
            dataStorage.telemetry.addData("folding", foldingSequence);
            dataStorage.telemetry.addData("unfolding", unfoldingSequence);
            dataStorage.telemetry.addData("unfolding low basket", unfoldingSequenceLowBasket);
            dataStorage.telemetry.addData("intaking", intakingSequence);
            dataStorage.telemetry.addData("diffy pitch", pitch);
            dataStorage.telemetry.addData("diffy roll", roll);
            dataStorage.telemetry.addData("folding timer", foldingTimer);
            dataStorage.telemetry.addData("intaking timer", intakingTimer);
            dataStorage.telemetry.addData("scoring mode", scoring_mode);
        }
    }
}
