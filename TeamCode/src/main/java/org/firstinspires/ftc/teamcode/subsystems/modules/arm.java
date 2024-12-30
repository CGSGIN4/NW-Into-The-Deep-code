package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.opencv.core.Mat;

@Config
public class arm {
    /* ------------------ HARDWARE ------------------ */
    public DcMotorEx rotationMotor;
    public DcMotorEx extensionMotor;
    public AnalogInput rotationBtn;

    /* ------------------ CONSTANTS ------------------ */
    int EXTENSION_FULL = 1320; //370
    int EXTENSION_FRONT_MAX = 806;
    int EXTENSION_LOW_BASKET = 475;
    int EXTENSION_LOW_CHAMBER = 180;
    int EXTENSION_HIGH_CHAMBER = 278;
    int EXTENSION_YELLOW_1 = 700;
    int EXTENSION_YELLOW_2 = 778;
    int EXTENSION_YELLOW_3 = 673;
    int EXTENSION_SUPPORT = 158;
    int ROTATION_FRONT = 0;
    int ROTATION_BACK_HANG1 = 487; //508
    int ROTATION_LIFT = 487; //498 //was 467 before stopper
    int ROTATION_CHAMBER = 300;

    /* ------------------ ON-FLY ------------------ */
    public int targetRotationPos;
    public int targetExtensionPos;
    public extension extensionState = extension.CLOSED;
    public rotation rotationState = rotation.FRONT;
    public rotation wantedRotation = rotation.FRONT;
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
    /* load 1:4 */ public PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.0026, 0, 0.0019, -0.006);
    public PIDFCoefficients EXTENSION_PIDF = new PIDFCoefficients(0.0034, 0, 0.0034, 0.004);/*0.017*/
    int oldExtensionError = 0;
    public double oldRotationError = 0;
    double rotDeltaRaw = 0;
    double rotDeltaFiltered = 0;
    int extensionSum = 0;
    double extDelta = 0;

    /* ------------------ PHYSICAL MODEL ------------------ */
    //public final double m_1 = 0.05;
    public final double m_1 = 0.531;
    public final double m_2 = 0.126;
    public final double m_3 = 0.126;
    public final double m_4 = 0.126;
    public final double m_5 = 0.4554;
    //public double l_1 = 0.135;
    public double l_1 = 0.214232;
    public double l_2 = 0.135;
    public double l_3 = 0.135;
    public double l_4 = 0.135;
    public double l_5 = 0.041;
    //public final double h_1 = 0.068;
    public final double h_1 = 0.093806;
    public final double h_2 = 0.0605;
    public final double h_3 = 0.0465;
    public final double h_4 = 0.0325;
    public final double h_5 = 0.0107;

    final double COG_MAX = 0.53;
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
        YELLOW_2,
        YELLOW_3,
        SUPPORT,
        PID_MANUAL
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
        rotationMotor = HM.get(DcMotorEx.class, "armRotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetRotationEncoders();

        extensionMotor = HM.get(DcMotorEx.class, "armExtensionMotor");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetExtensionEncoders();

        rotationBtn = HM.get(AnalogInput.class, "rotationBtn");
    }

    public void resetRotationEncoders() {
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetExtensionEncoders() {
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("rot state", rotationState);
        telemetry.addData("wanted rot state", wantedRotation);
        telemetry.addData("voltage rot", rotationMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("target ext", targetExtensionPos);
        telemetry.addData("ext state", extensionState);
        telemetry.addData("voltage ext", extensionMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("ext pos", extensionMotor.getCurrentPosition());
        cycleTime = cycleTimer.milliseconds();
        cycleTimer.reset();

        rotationAngle = getRotationAngle();
        extensionLen = getExtensionLength();

        cog = (m_1 * Math.sqrt(l_1 * l_1 + h_1 * h_1) + m_2 * Math.sqrt(l_2 * l_2 + h_2 * h_2) + m_3 * Math.sqrt(l_3 * l_3 + h_3 * h_3) + m_4 * Math.sqrt(l_4 * l_4 + h_4 * h_4) + m_5 * Math.sqrt(l_5 * l_5 + h_5 * h_5)) / (m_1 + m_2 + m_3 + m_4 + m_5);
        ratio = cog / COG_MAX;

        if (!extensionState.equals(extension.MANUAL) && !extensionState.equals(extension.PID_MANUAL)) {
            telemetry.addData("ext pwr", setExtension(extensionState));
        }
        if (!rotationState.equals(rotation.MANUAL))
            setRotation(wantedRotation);

        if (extensionState.equals(extension.PID_MANUAL))
            pidExtend(targetExtensionPos);

        if (rotationBtn.getVoltage() < 0.4 && (rotationState == rotation.FRONT || rotationState == rotation.MANUAL)) /* pressed */ {
            resetRotationEncoders();
            rotationState = rotation.RESET;
            setRotationMotorPower(0);
        }
        //telemetry.update();
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
                return (int)(-(31.5 / 928) * rotationMotor.getCurrentPosition());
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
            case YELLOW_3:
                return EXTENSION_YELLOW_3;
            case SUPPORT:
                return EXTENSION_SUPPORT;
        }
        return -1;
    }

    private int rotationPosToTicks(rotation target)
    {
        switch (target)
        {
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
        //if (extensionState == extension.EXTENDED && extensionMotor.getCurrentPosition() < EXTENSION_FULL)
          //  return 1;
        int error = targetPos - extensionMotor.getCurrentPosition();
        extDelta = error - oldExtensionError;

        if (Math.abs(error) < 40)
            extensionSum += error;

        if (error * oldExtensionError <= 0)
            extensionSum = 0;

        oldExtensionError = error;

        if (rotationState == rotation.LIFT && extensionState == extension.CLOSED && Math.abs(error) < 60 && Math.abs(error) > 10) {
            error /= 2;
        }

        return (error * EXTENSION_PIDF.p + extDelta * EXTENSION_PIDF.d + EXTENSION_PIDF.f * Math.cos(Math.toRadians(rotationAngle)) * ratio);
    }

    public double pidCalculateRotationPower(int targetPos){
        double error = targetPos - rotationMotor.getCurrentPosition();
        double k = 0.5;
        rotDeltaRaw = error - oldRotationError; /* negative on moving front -> back, positive back -> front */
        rotDeltaFiltered = rotDeltaRaw * k + rotDeltaFiltered * (1 - k);

        double delta = rotDeltaFiltered;
        velocity = (oldRotationError - error) / cycleTime;

        oldRotationError = error;

        if (Math.abs(rotationAngle) < 20 && rotationState == rotation.LIFT && extensionState == extension.EXTENDED) {
            if (Math.abs(error) > Math.abs(oldRotationError) && Math.abs(rotationAngle) > 2) /* error increased => wrong direction */
                delta *= 1.42;

            if (Math.abs(error) < 40)
                delta *= 2;
            else if (Math.abs(error) < 80)
                delta *= 2;
            else if (Math.abs(error) < 120)
                delta *= 2;
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

        if (Math.abs(rotationPosToTicks(rotationState) - rotationMotor.getCurrentPosition()) > 120 && targetExtensionPos != EXTENSION_SUPPORT)
            return this.setExtensionMotorPower(-0.3);
        else if (rotationState != rotation.LIFT || rotationReached())
            return this.setExtensionMotorPower(pidCalculateExtensionPower(targetExtensionPos));
        else
            return 0;
    }

    public double setExtensionInTicks(int target)
    {
        this.extensionState = extension.MANUAL;
        this.targetExtensionPos = target;

        return this.setExtensionMotorPower(pidCalculateExtensionPower(target));
    }

    public double setRotation(rotation target)
    {
        wantedRotation = target;
        if (rotationState == rotation.RESET)
        {
            resetRotationEncoders();
        }

        if (target == rotation.BACK_HANG1 || rotationState == rotation.BACK_HANG1 || (extensionState == extension.CLOSED && extensionMotor.getCurrentPosition() < 250) || (extensionMotor.getCurrentPosition() < 250 && extensionState == extension.MANUAL) || (extensionState == extension.SUPPORT && extensionMotor.getCurrentPosition() < 250)) {
            rotationState = wantedRotation;
            this.targetRotationPos = rotationPosToTicks(target);
        }

        if (target == rotation.LIFT && rotationMotor.getCurrentPosition() > ROTATION_LIFT - 30)
            return setRotationMotorPower(0.005);

        return this.setRotationMotorPower(pidCalculateRotationPower(targetRotationPos));
    }

    public void manuallyExtend(double speed){
        targetExtensionPos = extensionMotor.getCurrentPosition();
        if (rotationState == rotation.LIFT || this.extensionMotor.getCurrentPosition() < extensionPosToTicks(extension.FRONTAL_EXTENSION) || speed < 0) {
            this.extensionState = extension.MANUAL;

            if (rotationState == rotation.RESET)
                setExtensionMotorPower(speed * 0.75);
            else
                setExtensionMotorPower(speed);
        }
        else
            setExtension(extension.FRONTAL_EXTENSION);
    }

    public void pidExtend(int target){
        extensionState = extension.PID_MANUAL;
        targetExtensionPos = target;
        setExtensionMotorPower(pidCalculateExtensionPower(target));
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
        double k = 0.715 / 1320;
        double b = 0.3;

        l_1 = 0.180 + k * pos;
        //l_1 = 0.135 + 0.0018 * pos;
        if (pos > 337)
            l_2 = 0.135 + k * (pos - 337);
        else
            l_2 = 0.135;
        if (pos > 674)
            l_3 = 0.135 + k * (pos - 674);
        else
            l_3 = 0.135;
        if (pos > 1011)
            l_4 = 0.135 + k * (pos - 1011);
        else
            l_4 = 0.135;

        return k * pos + b;
    }

    public boolean extensionReached()
    {
        return Math.abs(extDelta) <= 5 && ((extensionMotor.getCurrentPosition() >= EXTENSION_FULL - 25 && extensionState == extension.EXTENDED) ||
                (extensionMotor.getCurrentPosition() < 13 && extensionState == extension.CLOSED) ||
                Math.abs(targetExtensionPos - extensionMotor.getCurrentPosition()) < 13);
    }

    public boolean rotationReached()
    {
        return Math.abs(targetRotationPos - rotationMotor.getCurrentPosition()) < 7 /* && rotDeltaFiltered < 4*/;
    }
}
