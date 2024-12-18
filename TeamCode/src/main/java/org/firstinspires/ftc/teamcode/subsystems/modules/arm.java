package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

@Config
public class arm {
    /* ------------------ HARDWARE ------------------ */
    public DcMotor rotationMotor;
    public DcMotor extensionMotor;
    public AnalogInput rotationBtn;

    /* ------------------ CONSTANTS ------------------ */
    int EXTENSION_FULL = 510; //370
    int EXTENSION_FRONT_MAX = 305;
    int EXTENSION_LOW_BASKET = 180;
    int EXTENSION_LOW_CHAMBER = 180;
    int EXTENSION_HIGH_CHAMBER = 278;
    int EXTENSION_YELLOW_1 = 305;
    int EXTENSION_YELLOW_2 = 278;
    int ROTATION_FRONT = 0;
    int ROTATION_BACK_HANG0 = 560;
    int ROTATION_BACK_HANG1 = 516;
    int ROTATION_LIFT = 498; //485
    int ROTATION_CHAMBER = 300;

    /* ------------------ ON-FLY ------------------ */
    public int targetRotationPos;
    int targetExtensionPos;
    public extension extensionState = extension.CLOSED;
    public rotation rotationState = rotation.FRONT;
    public double rotationAngle = -90;
    public double extensionLen = 0.3;
    ElapsedTime cycleTimer = new ElapsedTime();
    public double cycleTime = 0;
    public double velocity = 0;

    /* ------------------ CACHING ------------------ */
    double extPower = 0;
    double rotPower = 0;

    /* ------------------ PID ------------------ */
    /* no load 1:4 */ //public PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.0069, 0, 0.0003, -0.0064);
    /* no load 1:1.17 *///final PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.009, 0, 0.019, 0.028);
    /* load 1:1.17 *///final PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.02, 0, 0.04, 0.052);
    /* load 1:4 */ public PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.0058, 0, 0.0009, -0.006);
    public PIDFCoefficients EXTENSION_PIDF = new PIDFCoefficients(0.011, 0, 0.02, 0.01);
    int oldExtensionError = 0;
    public double oldRotationError = 0;
    double rotDeltaRaw = 0;
    double rotDeltaFiltered = 0;
    int extensionSum = 0;

    /* ------------------ PHYSICAL MODEL ------------------ */
    //public final double m_1 = 0.05;
    public final double m_1 = 0.527672;
    public final double m_2 = 0.126;
    public final double m_3 = 0.126;
    public final double m_4 = 0.126;
    public final double m_5 = 0.4554;
    //public double l_1 = 0.135;
    public double l_1 = 0.180;
    public double l_2 = 0.135;
    public double l_3 = 0.135;
    public double l_4 = 0.135;
    public double l_5 = 0.041;
    //public final double h_1 = 0.068;
    public final double h_1 = 0.07448;
    public final double h_2 = 0.0605;
    public final double h_3 = 0.0465;
    public final double h_4 = 0.0325;
    public final double h_5 = 0.0107;

    final double COG_MAX = 0.50;
    double cog = 0.12;
    double ratio = 0.3;

    /* ------------------ POSITIONS ------------------ */

    public enum extension {
        EXTENDED,
        LOW_BASKET,
        FRONTAL_EXTENSION,
        CLOSED,
        LOW_CHAMBER,
        HIGH_CHAMBER,
        MANUAL,
        YELLOW_1,
        YELLOW_2
    }

    public enum rotation {
        FRONT,
        BACK_HANG0,
        BACK_HANG1,
        CHAMBER,
        LIFT,
        RESET,
        MANUAL
    }
    public arm(HardwareMap HM){
        rotationMotor = HM.get(DcMotor.class, "armRotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetRotationEncoders();

        extensionMotor = HM.get(DcMotor.class, "armExtensionMotor");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetExtensionEncoders();

        rotationBtn = HM.get(AnalogInput.class, "rotationBtn");
    }

    private void resetRotationEncoders() {
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetExtensionEncoders() {
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(Telemetry telemetry) {
        cycleTime = cycleTimer.milliseconds();
        cycleTimer.reset();

        rotationAngle = getRotationAngle();
        extensionLen = getExtensionLength();

        cog = (m_1 * Math.sqrt(l_1 * l_1 + h_1 * h_1) + m_2 * Math.sqrt(l_2 * l_2 + h_2 * h_2) + m_3 * Math.sqrt(l_3 * l_3 + h_3 * h_3) + m_4 * Math.sqrt(l_4 * l_4 + h_4 * h_4) + m_5 * Math.sqrt(l_5 * l_5 + h_5 * h_5)) / (m_1 + m_2 + m_3 + m_4 + m_5);
        ratio = cog / COG_MAX;

        if (!extensionState.equals(extension.MANUAL))
            telemetry.addData("extension power", setExtension(extensionState));
        if (!rotationState.equals(rotation.MANUAL))
            telemetry.addData("rotation power", setRotation(rotationState));

        if (rotationBtn.getVoltage() < 0.4 && rotationState == rotation.FRONT) /* pressed */ {
            resetRotationEncoders();
            rotationState = rotation.RESET;
            setRotationMotorPower(0);
        }
    }

    public void update() {
        update(new MultipleTelemetry());
    }

    private int extensionPosToTicks(extension target)
    {
        switch (target)
        {
            case EXTENDED:
                return EXTENSION_FULL;
            case LOW_BASKET:
                return EXTENSION_LOW_BASKET;
            case CLOSED:
                return (int)(-(17.0 / 928) * rotationMotor.getCurrentPosition());
            case LOW_CHAMBER:
                return EXTENSION_LOW_CHAMBER;
            case HIGH_CHAMBER:
                return EXTENSION_HIGH_CHAMBER;
            case FRONTAL_EXTENSION:
                return EXTENSION_FRONT_MAX;
            case YELLOW_1:
                return EXTENSION_YELLOW_1;
            case YELLOW_2:
                return EXTENSION_YELLOW_2;
        }
        return -1;
    }

    private int rotationPosToTicks(rotation target)
    {
        switch (target)
        {
            case BACK_HANG0:
                return ROTATION_BACK_HANG0;
            case BACK_HANG1:
                return ROTATION_BACK_HANG1;
            case LIFT:
                return ROTATION_LIFT;
            case CHAMBER:
                return ROTATION_CHAMBER;
            case FRONT:
                return ROTATION_FRONT;
            default:
                return -1;
        }
    }

    private double setExtensionMotorPower(double power){
        if (Math.abs(extPower - power) > 0.00001) {
            this.extPower = power;
            this.extensionMotor.setPower(extPower);
        }
        return extPower;
    }

    private double setRotationMotorPower(double power){
        if (Math.abs(rotPower - power) > 0.001) {
            this.rotPower = power;
            this.rotationMotor.setPower(rotPower);
        }
        return rotPower;
    }

    private double pidCalculateExtensionPower(int targetPos){
        if (extensionState == extension.EXTENDED)
            return 1;
        int error = targetPos - extensionMotor.getCurrentPosition();
        int delta = error - oldExtensionError;

        if (Math.abs(error) < 40)
            extensionSum += error;

        if (error * oldExtensionError <= 0)
            extensionSum = 0;

        oldExtensionError = error;

        if (rotationState == rotation.LIFT && extensionState == extension.CLOSED && Math.abs(error) < 60 && Math.abs(error) > 10) {
            error /= 2;
        }

        return (error * EXTENSION_PIDF.p + delta * EXTENSION_PIDF.d + EXTENSION_PIDF.f * Math.cos(Math.toRadians(rotationAngle)) * ratio);
    }

    public double pidCalculateRotationPower(int targetPos){
        double error = targetPos - rotationMotor.getCurrentPosition();
        double k = 0.5;
        rotDeltaRaw = error - oldRotationError; /* negative on moving front -> back, positive back -> front */
        rotDeltaFiltered = rotDeltaRaw * k + rotDeltaFiltered * (1 - k);

        double delta = rotDeltaFiltered;
        velocity = (oldRotationError - error) / cycleTime;

        oldRotationError = error;

        if (rotationState == rotation.FRONT && rotationAngle < -50)
            error /= 3;

        if (Math.abs(rotationAngle) < 20 && rotationState == rotation.LIFT && extensionState == extension.EXTENDED) {
            if (Math.abs(error) > Math.abs(oldRotationError) && Math.abs(rotationAngle) > 2) /* error increased => wrong direction */
                delta *= 3;

            if (Math.abs(rotDeltaFiltered) > 13)
            {
                setExtension(extension.HIGH_CHAMBER);
            }
            if (Math.abs(error) < 40)
                delta *= 17;
            else if (Math.abs(error) < 80)
                delta *= 22;
            else if (Math.abs(error) < 120)
                delta *= 25;
            double modExponential = 0.00048759 * Math.pow(Math.abs(error), 1.461020);
            if (error > 0)
                error = modExponential / ROTATION_PIDF.p;
            else 
                error = -modExponential / ROTATION_PIDF.p;
        }

        return (error * ROTATION_PIDF.p + delta * ROTATION_PIDF.d + ROTATION_PIDF.f * Math.sin(Math.toRadians(rotationAngle)) * (1 + ratio * ratio));
    }
    public double setExtension(extension target)
    {
        this.targetExtensionPos = extensionPosToTicks(target);
        this.extensionState = target;

        if (Math.abs(rotationPosToTicks(rotationState) - rotationMotor.getCurrentPosition()) > 120)
            return this.setExtensionMotorPower(-0.2);
        else
            return this.setExtensionMotorPower(pidCalculateExtensionPower(targetExtensionPos));
    }

    public double setExtensionInTicks(int target)
    {
        this.extensionState = extension.MANUAL;

        return this.setExtensionMotorPower(pidCalculateExtensionPower(target));
    }

    public double setRotation(rotation target)
    {
        if (rotationState == rotation.RESET)
        {
            resetRotationEncoders();
        }

        if (target == rotation.BACK_HANG1 || rotationState == rotation.BACK_HANG1 || extensionState == extension.CLOSED || (extensionMotor.getCurrentPosition() < 100 && extensionState == extension.MANUAL)) {
            this.targetRotationPos = rotationPosToTicks(target);
            this.rotationState = target;
        }

        return this.setRotationMotorPower(pidCalculateRotationPower(targetRotationPos));
    }

    public void manuallyExtend(double speed){
        if (rotationState == rotation.LIFT || this.extensionMotor.getCurrentPosition() < extensionPosToTicks(extension.FRONTAL_EXTENSION) || speed < 0) {
            this.extensionState = extension.MANUAL;

            if (rotationState == rotation.RESET)
                setExtensionMotorPower(speed / 2);
            else
                setExtensionMotorPower(speed);
        }
        else
            setExtension(extension.FRONTAL_EXTENSION);
    }

    public void manuallyRotate(double speed){
        this.rotationState = rotation.MANUAL;
        setRotationMotorPower(speed);
    }

    public void stop(){
        this.extensionMotor.setPower(0);
        this.rotationMotor.setPower(0);
    }

    public double getRotationAngle(){
        double k = 180 / 928.0;
        double b = -90;

        return k * rotationMotor.getCurrentPosition() + b;
    }

    public double getExtensionLength(){
        double pos = extensionMotor.getCurrentPosition();
        double k = 0.715 / 500;
        double b = 0.3;

        l_1 = 0.180 + k * pos;
        //l_1 = 0.135 + 0.0018 * pos;
        if (pos > 126)
            l_2 = 0.135 + k * (pos - 126);
        else
            l_2 = 0.135;
        if (pos > 252)
            l_3 = 0.135 + k * (pos - 252);
        else
            l_3 = 0.135;
        if (pos > 378)
            l_4 = 0.135 + k * (pos - 378);
        else
            l_4 = 0.135;

        return k * pos + b;
    }

    public boolean extensionReached()
    {
        return (extensionMotor.getCurrentPosition() >= targetExtensionPos && extensionState == extension.EXTENDED) ||
                ((Math.abs(targetExtensionPos - extensionMotor.getCurrentPosition()) < 21) && extensionState == extension.CLOSED) ||
                Math.abs(targetExtensionPos - extensionMotor.getCurrentPosition()) < 5;
    }

    public boolean rotationReached()
    {
        return Math.abs(targetRotationPos - rotationMotor.getCurrentPosition()) < 7 /* && rotDeltaFiltered < 4*/;
    }
}
