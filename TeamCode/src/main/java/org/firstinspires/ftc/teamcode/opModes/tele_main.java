package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.FRONTAL_EXTENSION;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.LOW_BASKET;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.SUPPORT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.BACK_HANG1;
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

@TeleOp
@Config
public class tele_main extends LinearOpMode {
    double TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING = 0;
    int scoring_mode = 0;
    double scored = 0;
    public static double pitch = 176;
    public static double roll = -2;
    Vector2d gamepad = new Vector2d();
    double turn = 0;
    FtcDashboard dashboard;
    boolean foldingSequence = false;
    boolean unfoldingSequence = false;
    boolean unfoldingSequenceLowBasket = false;
    boolean intakingSequence = false;
    boolean diff_flip = false;
    boolean fast_pitch_swap = false;
    String lastCall;
    ElapsedTime foldingTimer = new ElapsedTime();
    ElapsedTime intakingTimer = new ElapsedTime();
    ElapsedTime safetyDiffTimer = new ElapsedTime();
    path_follower path_follower;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        path_follower = new path_follower(robot.drivetrain);
        velocity_calculator velocity_calculator = new velocity_calculator();

        differential differential = new differential(hardwareMap);
        arm arm = new arm(hardwareMap);
        hang hang = new hang(hardwareMap);

        robot.init();
        dataStorage.init(robot.drive, telemetry, this);

        GamepadNW driverGamepad = new GamepadNW(gamepad1);
        GamepadNW assistGamepad = new GamepadNW(gamepad2);

        waitForStart();
        robot.drive.setPoseEstimate(new Pose2d(dataStorage.RobotWorldX, dataStorage.RobotWorldY, transfer.angle));
        differential.closeClaw();
        while(opModeIsActive()) {
            driverGamepad.update();
            assistGamepad.update();
            dataStorage.updateData();
            arm.update(dataStorage.telemetry);
            hang.update();
            differential.update();

            /* DIFFERENTIAL SECTION */
            if (assistGamepad.isClicked("dpad_down")) {
                if (fast_pitch_swap) {
                    pitch = 13;
                    fast_pitch_swap = false;
                }
                if (pitch > 100)
                    pitch = 100;
                else
                    pitch = 13;
                if (arm.rotationState == RESET)
                    differential.openClaw();
            }
            if (assistGamepad.isClicked("dpad_up"))
                if (pitch < 100)
                    pitch = 100;
                else
                    pitch = 176;

            if (assistGamepad.isClicked("dpad_left"))
                if (roll >= -80 && roll <= -45)
                    roll = -80;
                else if (roll > -45 && roll <= -2)
                    roll = -45;
                else if (roll > -2 && roll <= 45)
                    roll = -2;
                else
                    roll = 45;
            if (assistGamepad.isClicked("dpad_right"))
                if (roll <= 75 && roll >= 45)
                    roll = 75;
                else if (roll >= -2 && roll < 45)
                    roll = 45;
                else if (roll >= -45 && roll < -2)
                    roll = -2;
                else
                    roll = -45;

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

                if (Math.abs(gamepad1.right_stick_y) > 0.05 || Math.abs(gamepad1.right_stick_x) > 0.05)
                    robot.drivetrain.applyVector(gamepad.div(2), turn / 2);
                else
                    robot.drivetrain.applyVector(gamepad, turn);
            } else
                robot.drivetrain.applyVector(new Vector2d(0, 0), 0);


            /* ARM SECTION */
            if (assistGamepad.isClicked("a")) {
                if (arm.rotationState == LIFT) {
                    if (arm.extensionMotor.getCurrentPosition() < 435) {
                        arm.setRotation(FRONT);
                        arm.setExtension(CLOSED);
                        lastCall = ("called by pressing X in rotation LIFT");
                        pitch = 176;
                    }
                }
                else {
                    arm.setRotation(LIFT);
                    arm.setExtension(SUPPORT);
                    lastCall = ("called by pressing X in rotation RESET");
                    pitch = 13;
                    roll = -2;
                }
                //tele.addData("rotation power", arm.setRotation(LIFT));
            }
            if (assistGamepad.isClicked("right_bumper")) {
                if (arm.rotationState == LIFT) {
                    if (arm.extensionMotor.getCurrentPosition() < 280) {
                        unfoldingSequenceLowBasket = false;
                        unfoldingSequence = true;
                    } else {
                        pitch = 125;
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

            if (assistGamepad.isClicked("b")) {
                scoring_mode = (scoring_mode == 1 ? 0 : 1);
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
                if (foldingTimer.milliseconds() < TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 500 && arm.extensionState != MANUAL)
                    arm.setExtension(EXTENDED);
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING) /* perekid */
                    differential.openClaw();
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 200 && !diff_flip) {
                    pitch = 13;
                    safetyDiffTimer.reset();
                    diff_flip = true;
                }
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 500 && safetyDiffTimer.milliseconds() > 300 && diff_flip) {
                    if (arm.extensionMotor.getCurrentPosition() > 100) {
                        arm.setExtension(CLOSED);
                        lastCall = ("called by folding sequence");
                    }
                    else if (arm.rotationMotor.getCurrentPosition() > 100) {
                        arm.setRotation(FRONT);
                        pitch = 176;
                    }
                    else {
                        roll = -2;
                        foldingSequence = false;
                        diff_flip = false;
                        fast_pitch_swap = true;
                    }
                }
            }

            if (unfoldingSequence) {
                unfoldingSequenceLowBasket = false;
                if (Math.abs(gamepad2.right_stick_y) > 0.01)
                    unfoldingSequence = false;

                if (scoring_mode == 1)
                    pitch = 13;
                else
                    pitch = 125;
                arm.setExtension(EXTENDED);
                lastCall = ("called by unfolding sequence");
                if (arm.extensionMotor.getCurrentPosition() > 1320) {
                    unfoldingSequence = false;
                }
            }

            if (unfoldingSequenceLowBasket) {
                unfoldingSequence = false;
                if (Math.abs(gamepad2.right_stick_y) > 0.01)
                    unfoldingSequenceLowBasket = false;

                if (scoring_mode == 1)
                    pitch = 13;
                else
                    pitch = 125;
                arm.setExtension(LOW_BASKET);
                lastCall = ("called by unfolding to low basket");
                if (arm.extensionMotor.getCurrentPosition() > 465)
                    unfoldingSequenceLowBasket = false;
            }

            if (intakingSequence) {
                if (intakingTimer.milliseconds() < 200)
                    differential.closeClaw();
                else {
                    arm.setExtension(CLOSED);
                    lastCall = ("called by intaking sequence");
                    pitch = 176;
                    intakingSequence = false;
                }
            }

            if (Math.abs(gamepad2.right_stick_y) > 0.01 && safetyDiffTimer.milliseconds() > 300) {
                arm.manuallyExtend(-gamepad2.right_stick_y);
                lastCall = ("called manually");
                //if (gamepad2.right_stick_y < -0.1 && arm.rotationState == RESET)
                    //differential.openClaw();
            }
            else {
                if (arm.extensionState == MANUAL) {
                    arm.extensionMotor.setPower(0);
                    lastCall = ("called by arm being in manual but not pressing stick");
                }
            }

            if (arm.rotationState == RESET) {
                if (gamepad2.right_trigger > 0.05 && arm.extensionMotor.getCurrentPosition() < 305) {
                    arm.pidExtend(arm.targetExtensionPos + 7);
                    lastCall = ("called by trigger");
                }
                if (gamepad2.left_trigger > 0.05) {
                    arm.pidExtend(arm.targetExtensionPos - 7);
                    lastCall = ("called by trigger down");
                }
            }

            if (assistGamepad.isClicked("y")) {
                arm.setRotation(LIFT);
                arm.setExtension(CLOSED);
                lastCall = ("called by hang");
                differential.openClaw();
                pitch = 176;
                if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING)
                    hang.prepare();
                else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.ASCEND2COMPLETE || hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.FOLDING)
                    hang.prepare3();
                else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.PREPARE3)
                    hang.standby3();
                else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.STANDBY3)
                    hang.fold3();
            }
            if (assistGamepad.isClicked("b") && hang.state != org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING) {
                hang.chill();
                hang.state = org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING;
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                hang.setPower(-gamepad2.left_stick_y);
                hang.state = org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING;
            }
            else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING)
                hang.setPower(0);

            if (assistGamepad.isClicked("start"))
                arm.resetExtensionEncoders();
            if (assistGamepad.isClicked("back"))
                arm.resetRotationEncoders();
            if (assistGamepad.isPressed("left_stick_button"))
                arm.manuallyRotate(-0.5);

            dataStorage.telemetry.addLine(lastCall);
            //dataStorage.telemetry.addLine("-----------TELEOP----------");
            //dataStorage.telemetry.addData("folding", foldingSequence);
            //dataStorage.telemetry.addData("unfolding", unfoldingSequence);
            //dataStorage.telemetry.addData("unfolding low basket", unfoldingSequenceLowBasket);
            //dataStorage.telemetry.addData("intaking", intakingSequence);
            //dataStorage.telemetry.addData("diffy pitch", pitch);
            //dataStorage.telemetry.addData("diffy roll", roll);
            //dataStorage.telemetry.addData("diff flip", diff_flip);
            //dataStorage.telemetry.addData("folding timer", foldingTimer);
            //dataStorage.telemetry.addData("safety timer", safetyDiffTimer);
            //dataStorage.telemetry.addData("intaking timer", intakingTimer);
            //dataStorage.telemetry.addData("scoring mode", scoring_mode);
        }
    }
}
