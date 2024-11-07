package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class arm {
    /* ------------------ HARDWARE ------------------ */
    public DcMotor rotationMotor;
    public DcMotor extensionMotor;
    public AnalogInput rotationBtn;

    /* ------------------ CONSTANTS ------------------ */
    int EXTENSION_FULL = 360; //370
    int EXTENSION_FRONT_MAX = 220;
    int EXTENSION_LOW_CHAMBER = 130;
    int EXTENSION_HIGH_CHAMBER = 200;
    int ROTATION_FRONT = 0;
    int ROTATION_BACK = 928;
    int ROTATION_LIFT = 928 / 2;
    int ROTATION_CHAMBER = 300;

    /* ------------------ ON-FLY ------------------ */
    public int targetRotationPos;
    int targetExtensionPos;
    public extension extensionState = extension.CLOSED;
    public rotation rotationState = rotation.FRONT;
    public double rotationAngle = -90;
    public double extensionLen = 0.3;

    /* ------------------ CACHING ------------------ */
    double extPower = 0;
    double rotPower = 0;

    /* ------------------ PID ------------------ */
    /* no load 1:4 */ //public PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.0069, 0, 0.0003, -0.0064);
    /* no load 1:1.17 *///final PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.009, 0, 0.019, 0.028);
    /* load 1:1.17 *///final PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.02, 0, 0.04, 0.052);
    /* load 1:4 */ public PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.0058, 0, 0.0003, -0.006);
    public PIDFCoefficients EXTENSION_PIDF = new PIDFCoefficients(0.007, 0, 0.02, 0.02);
    int oldExtensionError = 0;
    int oldRotationError = 0;
    int extensionSum = 0;
    int rotationSum = 0;

    /* ------------------ PHYSICAL MODEL ------------------ */
    //public final double m_1 = 0.05;
    public final double m_1 = 0.291762;
    public final double m_2 = 0.126;
    public final double m_3 = 0.126;
    public final double m_4 = 0.126;
    public final double m_5 = 0.4554;
    //public double l_1 = 0.135;
    public double l_1 = 0.218;
    public double l_2 = 0.135;
    public double l_3 = 0.135;
    public double l_4 = 0.135;
    public double l_5 = 0.041;
    //public final double h_1 = 0.068;
    public final double h_1 = 0.074;
    public final double h_2 = 0.0605;
    public final double h_3 = 0.0465;
    public final double h_4 = 0.0325;
    public final double h_5 = 0.0107;

    final double COG_MAX = 0.39;
    double cog = 0.12;
    double ratio = 0.3;

    /* ------------------ POSITIONS ------------------ */

    public enum extension {
        EXTENDED,
        FRONTAL_EXTENSION,
        CLOSED,
        LOW_CHAMBER,
        HIGH_CHAMBER,
        MANUAL
    }

    public enum rotation {
        FRONT,
        BACK,
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
        rotationAngle = getRotationAngle();
        extensionLen = getExtensionLength();

        cog = (m_1 * Math.sqrt(l_1 * l_1 + h_1 * h_1) + m_2 * Math.sqrt(l_2 * l_2 + h_2 * h_2) + m_3 * Math.sqrt(l_3 * l_3 + h_3 * h_3) + m_4 * Math.sqrt(l_4 * l_4 + h_4 * h_4) + m_5 * Math.sqrt(l_5 * l_5 + h_5 * h_5)) / (m_1 + m_2 + m_3 + m_4 + m_5);
        ratio = cog / COG_MAX;

        if (!extensionState.equals(extension.MANUAL))
            telemetry.addData("extension power", setExtension(extensionState));
        if (!rotationState.equals(rotation.MANUAL) && !rotationState.equals(rotation.RESET))
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
            case CLOSED:
                return (int)(-(33.0 / 928) * rotationMotor.getCurrentPosition());
            case LOW_CHAMBER:
                return EXTENSION_LOW_CHAMBER;
            case HIGH_CHAMBER:
                return EXTENSION_HIGH_CHAMBER;
            case FRONTAL_EXTENSION:
                return EXTENSION_FRONT_MAX;
        }
        return -1;
    }

    private int rotationPosToTicks(rotation target)
    {
        switch (target)
        {
            case BACK:
                return ROTATION_BACK;
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
        if (Math.abs(extPower - power) > 0.001) {
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
        int error = targetPos - extensionMotor.getCurrentPosition();
        int delta = error - oldExtensionError;

        if (Math.abs(error) < 40)
            extensionSum += error;

        if (error * oldExtensionError <= 0)
            extensionSum = 0;

        oldExtensionError = error;

        if (rotationState == rotation.LIFT && extensionState == extension.CLOSED && Math.abs(error) < 40)
            return 0;

        return (error * EXTENSION_PIDF.p + delta * EXTENSION_PIDF.d + EXTENSION_PIDF.f * Math.cos(Math.toRadians(rotationAngle)) * ratio);
    }

    public double pidCalculateRotationPower(int targetPos){
        int error = targetPos - rotationMotor.getCurrentPosition();
        int delta = error - oldRotationError;

        if (Math.abs(error) < 10)
            rotationSum += error;
        else
            rotationSum = 0;

        oldRotationError = error;

        if (rotationState == rotation.FRONT && rotationAngle < -50)
            error /= 3;

        if (Math.abs(rotationAngle) < 17 && rotationState == rotation.LIFT && extensionState == extension.EXTENDED) {
            delta *= 35;
            error *= 0.6;
        }

        return (error * ROTATION_PIDF.p + delta * ROTATION_PIDF.d + ROTATION_PIDF.f * Math.sin(Math.toRadians(rotationAngle)) * (1 + ratio * ratio));
    }
    public double setExtension(extension target)
    {
        this.targetExtensionPos = extensionPosToTicks(target);
        this.extensionState = target;

        if (Math.abs(rotationPosToTicks(rotationState) - rotationMotor.getCurrentPosition()) > 60)
            return this.setExtensionMotorPower(-0.2);
        else
            return this.setExtensionMotorPower(pidCalculateExtensionPower(targetExtensionPos));
    }

    public double setRotation(rotation target)
    {
        if (rotationState == rotation.RESET)
        {
            resetRotationEncoders();
        }

        if (extensionState == extension.CLOSED) {
            this.targetRotationPos = rotationPosToTicks(target);
            this.rotationState = target;
        }

        return this.setRotationMotorPower(pidCalculateRotationPower(targetRotationPos));
    }

    public void manuallyExtend(double speed){
        this.extensionState = extension.MANUAL;
        setExtensionMotorPower(speed);
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
        double k = 0.54 / 300;
        double b = 0.3;

        l_1 = 0.218 + 0.0018 * pos;
        //l_1 = 0.135 + 0.0018 * pos;
        if (pos > 100)
            l_2 = 0.135 + 0.0018 * (pos - 100);
        else
            l_2 = 0.135;
        if (pos > 200)
            l_3 = 0.135 + 0.0018 * (pos - 200);
        else
            l_3 = 0.135;
        if (pos > 300)
            l_4 = 0.135 + 0.0018 * (pos - 300);
        else
            l_4 = 0.135;

        return k * pos + b;
    }
}
