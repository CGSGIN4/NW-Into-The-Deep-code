package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class arm {
    /* ------------------ MOTORS ------------------ */
    DcMotor rotationMotor;
    DcMotor extensionMotor;

    /* ------------------ AFTER-INIT-CONSTANTS ------------------ */
    int initialExtensionTicks = 0;
    int initialRotationTicks = 0;
    int EXTENSION_FULL = 340;
    int EXTENSION_FRONT_MAX = 260;
    int EXTENSION_LOW_CHAMBER = 220;
    int EXTENSION_HIGH_CHAMBER = 280;
    int ROTATION_FRONT = 0;
    int ROTATION_BACK = 430;
    int ROTATION_LIFT = 280;

    /* ------------------ ON-FLY ------------------ */
    int targetRotationPos;
    int targetExtensionPos;
    extension extensionState;
    rotation rotationState;

    /* ------------------ PID ------------------ */
    final double ROTATION_P = 0;
    final double ROTATION_I = 0;
    final double ROTATION_D = 0;
    final double ROTATION_STATIC = 0;
    final double EXTENSION_P = 0;
    final double EXTENSION_I = 0;
    final double EXTENSION_D = 0;
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
        MANUAL
    }
    public arm(HardwareMap HM){
        rotationMotor = HM.get(DcMotor.class, "armRotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extensionMotor = HM.get(DcMotor.class, "armExtensionMotor");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    public void update(){
        if (!extensionState.equals(extension.MANUAL))
            setExtension(extensionState);
        if (!rotationState.equals(rotation.MANUAL))
            setRotation(rotationState);
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
        }
        return -1;
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

    private double pidCalculateRotationPower(int targetPos){
        int error = targetPos - rotationMotor.getCurrentPosition();
        int delta = error - oldRotationError;

        if (Math.abs(error) < 40)
            rotationSum += error;

        if (error * oldRotationError <= 0)
            rotationSum = 0;


        oldRotationError = error;
        return (error * ROTATION_P + delta * ROTATION_D + rotationSum * ROTATION_I);
    }
    public void setExtension(extension target)
    {
        this.targetExtensionPos = extensionPosToTicks(target);
        this.extensionState = target;
        this.setExtensionMotorPower(pidCalculateExtensionPower(targetExtensionPos));
    }

    public void setRotation(rotation target)
    {
        this.targetRotationPos = rotationPosToTicks(target);
        this.rotationState = target;
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
}
