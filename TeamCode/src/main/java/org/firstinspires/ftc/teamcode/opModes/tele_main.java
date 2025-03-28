package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENSION_SPEC_PREP;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.EXTENSION_SPEC_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.LOW_BASKET;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension.SUPPORT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.LIFT;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.RESET;
import static org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation.TAKE_SPEC;

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
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.modules.hang;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.velocity_calculator;
import org.firstinspires.ftc.teamcode.subsystems.vision.LLSmotritel;
import org.firstinspires.ftc.teamcode.utils.GamepadNW;
import org.firstinspires.ftc.teamcode.utils.logger;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.IOException;

@TeleOp(name = "!tele_main")
@Config
public class tele_main extends LinearOpMode {
    double TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING = 0;
    /** 0 - fast, 1 - slow **/
    int scoring_mode = 0;
    boolean scoring_samples = true;
    double scored = 0;
    public static double pitch = 176;
    public static double roll = -11;
    Vector2d gamepad = new Vector2d();
    double turn = 0;
    double extraWaitDiffyFlip = 0;
    FtcDashboard dashboard;
    boolean foldingSequence = false;
    boolean unfoldingSequence = false;
    boolean unfoldingSequenceLowBasket = false;
    boolean intakingSequence = false;
    boolean takeSpecSequence = false;
    boolean scoreSpecSequence = false;
    boolean fast_pitch_swap = false;
    boolean hang_called = false;
    boolean artemAutistReset = false;
    boolean autoScoreSpec = false;
    boolean takeSpecAutoCall = false;
    boolean scoreSpecAutoCall = false;
    boolean prepSpecAutoCall = false;
    boolean takeSpecAutoCallerReset = false;
    boolean scoreSpecAutoCallerReset = false;
    int reversedDrive = 1;

    public enum autoScoring {
        TAKE_SPEC,
        GO_TO_SUB_BEZIER,
        PREPARE_SPEC,
        GO_TO_SUB_PID,
        SCORE_SPEC,
        RESET_ARM,
        GO_TO_SPEC_BEZIER,
        GO_TO_SPEC_PID
    }
    autoScoring autoScoringState = autoScoring.TAKE_SPEC;
    String lastCall;
    ElapsedTime foldingTimer = new ElapsedTime();
    ElapsedTime intakingTimer = new ElapsedTime();
    ElapsedTime hangTimer = new ElapsedTime();
    ElapsedTime zazhimTimer = new ElapsedTime();
    ElapsedTime takeSpecTimer = new ElapsedTime();
    ElapsedTime artemAutist = new ElapsedTime();
    ElapsedTime autoScoringTimer = new ElapsedTime();
    path_follower path_follower;

    @Override
    public void runOpMode() throws InterruptedException {
        //logger.init();
        LLSmotritel smotritel = new LLSmotritel(hardwareMap, 0);
        smotritel.startStreaming();
        smotritel.stopStreaming();
        Robot robot = new Robot(hardwareMap);
        robot.init();
        parser parser = new parser("speciOp");
        dataStorage.init(robot.drive, telemetry, this);
        path_follower = new path_follower(robot.drivetrain);

        curve[] curves;
        try {
            curves = parser.getCurves();
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }

        differential differential = new differential(hardwareMap);
        arm arm = new arm(hardwareMap, true);
        hang hang = new hang(hardwareMap);

        dataStorage.opModeIsAutonomous = false;

        GamepadNW driverGamepad = new GamepadNW(gamepad1);
        GamepadNW assistGamepad = new GamepadNW(gamepad2);

        waitForStart();
        //logger.writeLn("started teleop");
        /* reset to init state */
        differential.closeClaw();
        zazhimTimer.reset();
        while(zazhimTimer.milliseconds() < 700);
        differential.pitchUp();


        robot.drive.setPoseEstimate(new Pose2d(dataStorage.RobotWorldX, dataStorage.RobotWorldY, transfer.angle));
/* -----------------------MAINLOOP----------------------- */
        while(opModeIsActive()) {
            driverGamepad.update();
            assistGamepad.update();
            dataStorage.updateData();
            arm.update(dataStorage.telemetry);
            hang.update(dataStorage.telemetry);
            differential.update(dataStorage.telemetry);

/* -----------------------DIFFERENTIAL CONTROL SECTION----------------------- */
            if (assistGamepad.isClicked("dpad_down")) {
                lastCall = "dpad";
                pitch = 6;
                if (arm.rotationState == RESET)
                    differential.openClaw();
            }
            if (assistGamepad.isClicked("dpad_up")) {
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

/* ------------------------DRIVE SECTION------------------------ */

            /* HYRO RESET */
            if (driverGamepad.isClicked("start"))
                robot.drive.setPoseEstimate(new Pose2d(dataStorage.RobotWorldX, dataStorage.RobotWorldY, -Math.PI));

            /* INVERT DRIVE */
            if (driverGamepad.isClicked("touchpad")) {
                reversedDrive *= -1;
                gamepad1.rumble(200);
            }

            /* DRIVE PROCESSING */
            if (Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.left_stick_y) > 0.01 || gamepad1.left_trigger > 0.01 || gamepad1.right_trigger > 0.01) {
                gamepad = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y).times(reversedDrive);
                autoScoreSpec = false;

                if (driverGamepad.isPressed("right_bumper") && scoring_samples)
                    turn = path_follower.velocity_calculator.getRotationCustomDirection(-Math.PI);
                else if (driverGamepad.isPressed("left_bumper") && scoring_samples)
                    turn = path_follower.velocity_calculator.getRotationCustomDirection(-Math.PI * 3 / 4);
                else if ((Math.abs(gamepad1.right_stick_y) > 0.05 || Math.abs(gamepad1.right_stick_x) > 0.05) && !scoring_samples)
                    turn = path_follower.velocity_calculator.getRotationCustomDirection(Math.PI / 2);
                else
                    turn = (gamepad1.left_trigger - gamepad1.right_trigger);

                if ((arm.extensionMotor.getCurrentPosition() > arm.EXTENSION_FRONT_MAX || arm.extensionState == LOW_BASKET && arm.extensionReached()) && arm.rotationState == LIFT)
                    turn *= 0.4;

                if ((Math.abs(gamepad1.right_stick_y) > 0.05 || Math.abs(gamepad1.right_stick_x) > 0.05) && scoring_samples)
                    gamepad = gamepad.div(2);

                robot.drivetrain.applyVector(gamepad, turn);
            }
            else if (!autoScoreSpec)
                robot.drivetrain.applyVector(new Vector2d(0, 0), 0);

/* ---------------------AUTO SCORING-------------------------*/

            /* AUTO SCORING EXECUTION */
            if (driverGamepad.isClicked("a")) {
                robot.drive.setPoseEstimate(new Pose2d(-41.237, 64.214, Math.PI / 2));
                autoScoreSpec = true;
                autoScoringState = autoScoring.TAKE_SPEC;
            }
/*
            if (autoScoreSpec) {
                if (autoScoringState == autoScoring.TAKE_SPEC) {
                    if (path_follower.goToPosTeleop(-41.237, 64.214, Math.PI / 2, false)) {
                        if (!takeSpecAutoCallerReset) {
                            prepSpecAutoCall = true;
                            takeSpecAutoCallerReset = true;
                            autoScoringTimer.reset();
                        }
                        if (autoScoringTimer.milliseconds() > 450) {
                            autoScoringState = autoScoring.GO_TO_SUB_PID;
                            takeSpecAutoCallerReset = false;
                        }
                    }
                }
                if (autoScoringState == autoScoring.GO_TO_SUB_PID)
                {
                    if (path_follower.goToPosTeleop(-0.926, 38.681, Math.PI / 2, false)) {
                        if (!scoreSpecAutoCallerReset) {
                            autoScoringTimer.reset();
                            scoreSpecAutoCall = true;
                            scoreSpecAutoCallerReset = true;
                        }
                        if (autoScoringTimer.milliseconds() > 450)
                        {
                            scoreSpecAutoCallerReset = false;
                            autoScoringState = autoScoring.GO_TO_SPEC_PID;
                        }
                    }
                }
                if (autoScoringState == autoScoring.GO_TO_SPEC_PID)
                    if (path_follower.goToPosTeleopBeforeIntake(-41.237, 64.214, Math.PI / 2, false))
                        autoScoringState = autoScoring.TAKE_SPEC;
            }
*/

/* --------------------SCORING SAMPLES-----------------------*/

            /* SWITCH MODES */
            if (assistGamepad.isClicked("touchpad")) {
                scoring_samples = !scoring_samples;
                if (scoring_samples)
                    gamepad2.rumble(500);
                else
                    gamepad2.rumble(200);
            }

/* --------------------SCORING SAMPLES KEYBINDS-----------------------*/
            if (assistGamepad.isClicked("a") && scoring_samples) {
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

            if (assistGamepad.isClicked("right_bumper") && scoring_samples) {
                if (arm.rotationState == LIFT) {
                    if (arm.extensionMotor.getCurrentPosition() + arm.offset < 280) {
                        unfoldingSequenceLowBasket = false;
                        unfoldingSequence = true;
                        zazhimTimer.reset();
                        //logger.writeLn("activated unfolding sequence");
                    } else {
                        if (pitch == 6)
                            TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING = 500;
                        pitch = 125;
                        lastCall = "folding seq";
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

            if (assistGamepad.isClicked("left_bumper") && arm.rotationState == LIFT && scoring_samples) {
                unfoldingSequenceLowBasket = true;
                unfoldingSequence = false;
                //logger.writeLn("activated unfoldingLowBasket sequence");
            }

            if (assistGamepad.isClicked("b") && scoring_samples) {
                scoring_mode = (scoring_mode == 1 ? 0 : 1);
                //logger.writeLn("scoring mode set to " + scoring_mode);
                if (scoring_mode == 1)
                    TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING = 500;
                else
                    TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING = 0;
            }

/* --------------------SCORING SAMPLES SEQUENCES-----------------------*/
            if (foldingSequence) {
                unfoldingSequenceLowBasket = false;
                unfoldingSequence = false;
                intakingSequence = false;
                if (foldingTimer.milliseconds() < TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 500 + extraWaitDiffyFlip && arm.extensionState != MANUAL && arm.extensionMotor.getCurrentPosition() > arm.EXTENSION_FRONT_MAX)
                    arm.setExtension(EXTENDED);
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING) /* perekid */ {
                    differential.openClaw();
                    if (roll != -11)
                        extraWaitDiffyFlip = 250;
                }
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 200) {
                    lastCall = "folding seq";
                    roll = -11;
                }
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 200 + extraWaitDiffyFlip)
                    pitch = 6;
                if (foldingTimer.milliseconds() > TIME_BETWEEN_DIFF_FLIP_AND_CLAW_OPENING + 500 + extraWaitDiffyFlip) {
                    if (arm.extensionMotor.getCurrentPosition() + arm.offset > 100) {
                        arm.setExtension(CLOSED);
                    }
                    else {
                        arm.setRotation(FRONT);
                        foldingSequence = false;
                        pitch = 176;
                        extraWaitDiffyFlip = 0;
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
                else if (!lastCall.equals("dpad_up")){
                    pitch = 125;
                    lastCall = "unfolding seq";
                }
                if (zazhimTimer.milliseconds() > 1000 && scoring_mode == 0)
                    differential.closeClawVerySilno();
                arm.setExtension(EXTENDED);
                if (arm.extensionMotor.getCurrentPosition() + arm.offset > 1320) {
                    unfoldingSequence = false;
                    if (scoring_mode == 0)
                        differential.closeClawVerySilno();
                    //logger.writeLn("unfolding sequence finished as planned");
                }
            }

            if (unfoldingSequenceLowBasket) {
                unfoldingSequence = false;
                if (Math.abs(gamepad2.right_stick_y) > 0.01)
                    unfoldingSequenceLowBasket = false;

                if (scoring_mode == 1)
                    pitch = 6;
                else {
                    pitch = 125;
                    lastCall = "unfolding seq low";
                }
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

/* --------------------ARM MANUAL CONTROL-------------------- */
            if (Math.abs(gamepad2.right_stick_y) > 0.01)
                arm.manuallyExtend(-gamepad2.right_stick_y);
            else {
                if (arm.extensionState == MANUAL && !assistGamepad.isPressed("right_stick_button")) {
                    arm.extensionMotor.setPower(0);
                }
            }

            if (arm.rotationState == RESET) {
                if (gamepad2.right_trigger > 0.05 && arm.extensionMotor.getCurrentPosition() + arm.offset < 305) {
                    arm.pidExtend(arm.targetExtensionPos + 5);
                }
                if (gamepad2.left_trigger > 0.05) {
                    arm.pidExtend(arm.targetExtensionPos - 5);
                }
            }
/* --------------------SCORING SPECIMENS-----------------------*/

/* --------------------SCORING SPECIMENS KEYBINDS-----------------------*/
            if (assistGamepad.isClicked("right_bumper") && !scoring_samples/* || takeSpecAutoCall || prepSpecAutoCall || scoreSpecAutoCall*/) {
                if (arm.rotationState == RESET && pitch != 30) {
                    pitch = 30;
                    differential.openClaw();
                    takeSpecAutoCall = false;
                }
                else if (arm.rotationState == RESET && pitch == 30) {
                    differential.closeClawSilno();
                    takeSpecSequence = true;
                    takeSpecTimer.reset();
                    prepSpecAutoCall = false;
                }
                else if (arm.rotationState == LIFT && arm.extensionState == EXTENSION_SPEC_PREP) {
                    arm.setExtension(EXTENSION_SPEC_SCORE);
                    scoreSpecSequence = true;
                    scoreSpecAutoCall = false;
                }
            }

            if (assistGamepad.isClicked("a") && !scoring_samples) {
                arm.setExtension(CLOSED);
                arm.setRotation(FRONT);
                pitch = 176;
                takeSpecSequence = false;
                scoreSpecSequence = false;
            }


/* --------------------SCORING SPECIMENS SEQUENCES-----------------------*/
            if (takeSpecSequence) {
                if (takeSpecTimer.milliseconds() > 200) {
                    pitch = 176;
                    arm.setExtension(CLOSED);
                }
                if (takeSpecTimer.milliseconds() > 450) {
                    arm.setRotation(LIFT);
                    arm.setExtension(EXTENSION_SPEC_PREP);
                }
                if (arm.rotationReached() && arm.rotationState == LIFT && !artemAutistReset) {
                    artemAutist.reset();
                    artemAutistReset = true;
                }
                if (artemAutistReset && artemAutist.milliseconds() > 300) {
                    differential.closeClawVerySilno();
                    takeSpecSequence = false;
                    artemAutistReset = false;
                }
            }

            /*
            if (scoreSpecSequence) {
                differential.closeClawVerySilno();
                if (arm.extensionMotor.getCurrentPosition() > 800) {
                    differential.openClaw();
                    scoreSpecSequence = false;
                    arm.setExtension(CLOSED);
                    arm.setRotation(TAKE_SPEC);
                    pitch = 80;
                }
            }
             */
            if (scoreSpecSequence) {
                differential.closeClawVerySilno();
                if (arm.extensionMotor.getCurrentPosition() > 800) {
                    differential.openClaw();
                    scoreSpecSequence = false;
                    arm.setExtension(CLOSED);
                    arm.setRotation(FRONT);
                    pitch = 30;
                }
            }

/* ---------------------------HANG-----------------------------*/
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
                else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.PREPARE3 || hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.READY3)
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
            if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.PREPARE3)
                pitch = 93;

            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                hang.setPower(-gamepad2.left_stick_y);
                hang.state = org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING;
                hang_called = false;
            }
            else if (hang.state == org.firstinspires.ftc.teamcode.subsystems.modules.hang.states.SLEEPING)
                hang.setPower(0);

/* ----------------------FAILSAFETY---------------------- */

            if (assistGamepad.isClicked("start"))
                arm.resetExtensionEncoders();
            if (assistGamepad.isClicked("back"))
                arm.resetRotationEncoders();
            if (assistGamepad.isPressed("left_stick_button"))
                arm.manuallyRotate(-0.5);

            if (assistGamepad.isPressed("right_stick_button")) {
                arm.manuallyRotate(-0.5);
                arm.manuallyExtend(-1);
                arm.resetRotationEncoders();
                arm.resetExtensionEncoders();
            }

/* ----------------------TELEMETRY---------------------- */
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
