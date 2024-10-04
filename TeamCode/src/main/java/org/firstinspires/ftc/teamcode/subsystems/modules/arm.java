package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class arm {
    /* ------------------ MOTORS ------------------ */
    public DcMotor rotationMotor;
    public DcMotor extensionMotor;
    public AnalogInput rotationBtn;

    /* ------------------ AFTER-INIT-CONSTANTS ------------------ */
    int initialExtensionTicks = 0;
    int initialRotationTicks = 0;
    int EXTENSION_FULL = 340;
    int EXTENSION_FRONT_MAX = 260;
    int EXTENSION_LOW_CHAMBER = 220;
    int EXTENSION_HIGH_CHAMBER = 280;
    int ROTATION_FRONT = 0;
    int ROTATION_BACK = 450;
    int ROTATION_LIFT = 280;

    /* ------------------ ON-FLY ------------------ */
    public int targetRotationPos;
    int targetExtensionPos;
    public extension extensionState = extension.CLOSED;
    public rotation rotationState = rotation.FRONT;
    public double rotationAngle = -90;
    public double extensionLen = 0.3;

    /* ------------------ PID ------------------ */
    final double ROTATION_P = 0.0026;
    final double ROTATION_I = 0;
    final double ROTATION_D = 0.02;
    final double BASKET_ROTATION_P = 0.004;
    final double BASKET_ROTATION_D = 0.06;
    final double EXTENSION_P = 0.02;
    final double EXTENSION_I = 0.0003;
    final double EXTENSION_D = 0.002;
    final double EXTENSION_STATIC = 0;
    int oldExtensionError = 0;
    int oldRotationError = 0;
    int extensionSum = 0;
    int rotationSum = 0;

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
        resetRotationEncoders();

        rotationBtn = HM.get(AnalogInput.class, "rotationBtn");
        /* prob dont need it */
        initialExtensionTicks = extensionMotor.getCurrentPosition();
        initialRotationTicks = rotationMotor.getCurrentPosition();

        EXTENSION_FULL = EXTENSION_FULL + initialExtensionTicks;
        EXTENSION_LOW_CHAMBER = EXTENSION_LOW_CHAMBER + initialExtensionTicks;
        EXTENSION_HIGH_CHAMBER = EXTENSION_HIGH_CHAMBER + initialExtensionTicks;
        EXTENSION_FRONT_MAX = EXTENSION_FRONT_MAX + initialExtensionTicks;

        ROTATION_BACK = ROTATION_BACK + initialRotationTicks;
        ROTATION_FRONT = ROTATION_FRONT + initialRotationTicks;
        ROTATION_LIFT = ROTATION_LIFT + initialRotationTicks;
    }

    private void resetRotationEncoders() {
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(){
        if (!extensionState.equals(extension.MANUAL))
            setExtension(extensionState);
        if (!rotationState.equals(rotation.MANUAL) && !rotationState.equals(rotation.RESET))
            setRotation(rotationState);
        rotationAngle = getRotationAngle();
        extensionLen = getExtensionLength();
        if (rotationBtn.getVoltage() < 0.4 && rotationState == rotation.FRONT) /* pressed */ {
            resetRotationEncoders();
            rotationState = rotation.RESET;
            rotationMotor.setPower(0);
        }
    }

    private int extensionPosToTicks(extension target)
    {
        switch (target)
        {
            case EXTENDED:
                return EXTENSION_FULL;
            case CLOSED:
                return initialExtensionTicks;
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
            case FRONT:
                return ROTATION_FRONT;
            default:
                return -1;
        }
    }

    private void setExtensionMotorPower(double power){
        this.extensionMotor.setPower(power);
    }

    private void setRotationMotorPower(double power){
        this.rotationMotor.setPower(power);
    }

    private double pidCalculateExtensionPower(int targetPos){
        int error = targetPos - extensionMotor.getCurrentPosition();
        int delta = error - oldExtensionError;

        if (Math.abs(error) < 40)
            extensionSum += error;

        if (error * oldExtensionError <= 0)
            extensionSum = 0;


        oldExtensionError = error;

        return (error * EXTENSION_P + delta * EXTENSION_D + extensionSum * EXTENSION_I);
    }

    public double pidCalculateRotationPower(int targetPos){
        int error = targetPos - rotationMotor.getCurrentPosition();
        int delta = error - oldRotationError;
        double kp, kd;
        //TODO: reorganise kG to be constant defined up in file
        double kG = 0.03;

        if (Math.abs(error) < 40)
            rotationSum += error;

        if (error * oldRotationError <= 0)
            rotationSum = 0;

        /*
        switch (rotationState){
            case LIFT:
                switch (extensionState){
                    case MANUAL:
                        kp = BASKET_ROTATION_P;
                        kd = BASKET_ROTATION_D;
                        break;
                    default:
                        kp = ROTATION_P;
                        kd = ROTATION_D;
                }
                break;
            default:
                kp = ROTATION_P;
                kd = ROTATION_D;
        }
        */

        kp = ROTATION_P;
        kd = ROTATION_D;

        oldRotationError = error;
        return (error * kp + delta * kd + kG * Math.sin(Math.toRadians(rotationAngle)) * extensionLen);
    }
    public void setExtension(extension target)
    {
        this.targetExtensionPos = extensionPosToTicks(target);
        this.extensionState = target;
        this.setExtensionMotorPower(pidCalculateExtensionPower(targetExtensionPos));
    }

    public void setRotation(rotation target)
    {
        if (rotationState == rotation.RESET)
        {
            resetRotationEncoders();
        }

        if (extensionState == extension.CLOSED) {
            this.targetRotationPos = rotationPosToTicks(target);
            this.rotationState = target;
        }
        this.setRotationMotorPower(pidCalculateRotationPower(targetRotationPos));
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
        double k = 18 / 45.0;
        double b = -90;

        return k * rotationMotor.getCurrentPosition() + b;
    }

    public double getExtensionLength(){
        double k = 0.7 / 340;
        double b = 0.3;

        return k * extensionMotor.getCurrentPosition() + b;
    }
}
