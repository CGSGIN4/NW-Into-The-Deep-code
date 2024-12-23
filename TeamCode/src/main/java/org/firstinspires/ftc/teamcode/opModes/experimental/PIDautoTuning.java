package org.firstinspires.ftc.teamcode.opModes.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;

@Config
@TeleOp(name = "PID Auto-Tuning", group = "TeleOp")
public class PIDautoTuning extends LinearOpMode {

    // Коэффициенты PID для движения
    public static double kP_move = 0.1;
    public static double kI_move = 0.0;
    public static double kD_move = 0.0;

    // Коэффициенты PID для поворота
    public static double kP_turn = 0.1;
    public static double kI_turn = 0.0;
    public static double kD_turn = 0.0;

    // Ограничения на максимальные значения коэффициентов
    public static final double KP_MAX = 1.0;
    public static final double KI_MAX = 0.1;
    public static final double KD_MAX = 0.5;

    // Параметры автонастройки
    public static final double ERROR_THRESHOLD = 0.05; // Допустимая ошибка перемещения
    public static final double OVERSHOOT_THRESHOLD = 0.1; // Порог переезда
    public static final double ANGLE_ERROR_THRESHOLD = 0.1; // Допустимая ошибка поворота (в радианах)

    private double previousErrorMove = 0.0;
    private double integralSumMove = 0.0;

    private double previousErrorTurn = 0.0;
    private double integralSumTurn = 0.0;

    private ElapsedTime timer = new ElapsedTime();
    Robot robot;
    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Improved PID Auto-Tuning...");
        telemetry.update();
        robot = new Robot(hardwareMap);
        robot.init();
        dataStorage.init(robot.drive, telemetry, this);

        waitForStart();

        Pose2d position1 = new Pose2d(24, 24, Math.PI);
        Pose2d position2 = new Pose2d(-24, -24, -Math.PI);
        Pose2d currentTarget = position1;

        while (opModeIsActive()) {
            boolean reachedTarget = moveToTargetWithRotation(currentTarget.vec(), currentTarget.getHeading()); // Угол поворота = 0

            if (reachedTarget) {
                currentTarget = (currentTarget.equals(position1)) ? position2 : position1;
            }

            telemetry.addData("Current Target", currentTarget);
            telemetry.addData("kP_move", kP_move);
            telemetry.addData("kI_move", kI_move);
            telemetry.addData("kD_move", kD_move);
            telemetry.addData("kP_turn", kP_turn);
            telemetry.addData("kI_turn", kI_turn);
            telemetry.addData("kD_turn", kD_turn);
            telemetry.update();
        }
    }

    private boolean moveToTargetWithRotation(Vector2d targetPosition, double targetAngle) {
        previousErrorMove = 0.0;
        integralSumMove = 0.0;

        previousErrorTurn = 0.0;
        integralSumTurn = 0.0;

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 5.0) {
            Vector2d currentPosition = getRobotPosition();
            double currentAngle = getRobotAngle();

            Vector2d errorVector = targetPosition.minus(currentPosition);
            double errorMagnitude = errorVector.norm();
            double angleError = normalizeAngle(targetAngle - currentAngle);

            if (errorMagnitude < ERROR_THRESHOLD && Math.abs(angleError) < ANGLE_ERROR_THRESHOLD && dataStorage.RobotVelocity.norm() < 3) {
                return true;
            }

            Vector2d controlVector = calculatePIDMove(errorMagnitude);
            double controlRotation = calculatePIDTurn(angleError);

            controlVector = limitVector(controlVector, 1.0);
            controlRotation = Math.max(-1.0, Math.min(1.0, controlRotation));

            applyVectorFieldCentric(controlVector, controlRotation);

            previousErrorMove = errorMagnitude;
            previousErrorTurn = angleError;

            tunePIDCoefficients(errorMagnitude, angleError);

            telemetry.addData("Error Magnitude", errorMagnitude);
            telemetry.addData("Angle Error", angleError);
            telemetry.update();
        }

        return false;
    }

    private void tunePIDCoefficients(double moveError, double turnError) {
        double deltaTime = timer.seconds();

        // Настройка коэффициентов движения

        // Уменьшаем kI, если ошибка колеблется вокруг цели
        if (Math.abs(moveError) < ERROR_THRESHOLD && Math.abs(previousErrorMove - moveError) < 0.01) {
            kI_move = Math.max(0.0, kI_move - 0.001);
        }

        // Уменьшаем kD, если наблюдаются колебания
        if (Math.abs(previousErrorMove - moveError) > OVERSHOOT_THRESHOLD) {
            kD_move = Math.max(0.0, kD_move - 0.01);
        }

        // Уменьшаем kD, если при маленькой ошибке робот неустойчив
        if (Math.abs(moveError) < ERROR_THRESHOLD && Math.abs(previousErrorMove - moveError) > 0.01) {
            kD_move = Math.max(0.0, kD_move - 0.01);
        }

        // Уменьшаем kD, если ошибка уменьшается линейно, но процесс слишком медленный
        if (Math.abs(moveError) > 0.1 && Math.abs(previousErrorMove - moveError) / deltaTime < 0.01) {
            kD_move = Math.max(0.0, kD_move - 0.01);
        }

        // Ограничиваем коэффициенты движения
        kP_move = Math.min(KP_MAX, kP_move);
        kI_move = Math.min(KI_MAX, kI_move);
        kD_move = Math.min(KD_MAX, kD_move);

        // Настройка коэффициентов поворота

        // Уменьшаем kI, если ошибка поворота стабильно мала
        if (Math.abs(turnError) < ANGLE_ERROR_THRESHOLD && Math.abs(previousErrorTurn - turnError) < 0.01) {
            kI_turn = Math.max(0.0, kI_turn - 0.001);
        }

        // Уменьшаем kD, если наблюдаются колебания поворота
        if (Math.abs(previousErrorTurn - turnError) > OVERSHOOT_THRESHOLD) {
            kD_turn = Math.max(0.0, kD_turn - 0.01);
        }

        // Уменьшаем kD, если поворот слишком медленный при маленькой ошибке
        if (Math.abs(turnError) > 0.1 && Math.abs(previousErrorTurn - turnError) / deltaTime < 0.01) {
            kD_turn = Math.max(0.0, kD_turn - 0.01);
        }

        // Ограничиваем коэффициенты поворота
        kP_turn = Math.min(KP_MAX, kP_turn);
        kI_turn = Math.min(KI_MAX, kI_turn);
        kD_turn = Math.min(KD_MAX, kD_turn);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private Vector2d limitVector(Vector2d vector, double maxLength) {
        if (vector.norm() > maxLength) {
            return vector.times(maxLength / vector.norm());
        }
        return vector;
    }

    private Vector2d calculatePIDMove(double error) {
        double deltaTime = timer.seconds();
        timer.reset();

        integralSumMove += error * deltaTime;
        double derivative = (error - previousErrorMove) / deltaTime;

        double output = kP_move * error + kI_move * integralSumMove + kD_move * derivative;
        return new Vector2d(output, output);
    }

    private double calculatePIDTurn(double error) {
        double deltaTime = timer.seconds();
        timer.reset();

        integralSumTurn += error * deltaTime;
        double derivative = (error - previousErrorTurn) / deltaTime;

        return kP_turn * error + kI_turn * integralSumTurn + kD_turn * derivative;
    }

    private Vector2d getRobotPosition() {
        return robot.drive.getPoseEstimate().vec();
    }

    private double getRobotAngle() {
        return robot.drive.getPoseEstimate().getHeading();
    }

    private void applyVectorFieldCentric(Vector2d vector, double rotation) {
        telemetry.addData("Vector applied", vector);
        telemetry.addData("Rotation applied", rotation);
    }
}
