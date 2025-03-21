package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.data.transfer;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.utils.logger;
import org.opencv.core.Mat;

@Config
public class arm {
    /* ------------------ HARDWARE ------------------ */
    public DcMotorEx rotationMotor;
    public DcMotorEx extensionMotor;
    public AnalogInput rotationBtn;

    /* ------------------ CONSTANTS ------------------ */
    boolean teleop = false;
    int EXTENSION_FULL = 1380; //370
    public int EXTENSION_FRONT_MAX = 730;
    int EXTENSION_LOW_BASKET = 475;
    int EXTENSION_LOW_CHAMBER = 180;
    int EXTENSION_HIGH_CHAMBER = 524;
    int EXTENSION_YELLOW_1 = 725;
    int EXTENSION_YELLOW_2 = 700;
    int EXTENSION_YELLOW_3 = 673;
    int EXTENSION_YELLOW_1_PRO = 525;
    int EXTENSION_YELLOW_2_PRO = 500;
    int EXTENSION_YELLOW_3_PRO = 130;
    int EXTENSION_DOBOR = 680;
    int EXTENSION_YELLOW_AFKBOT = 510;
    int EXTENSION_SUPPORT = 158;
    int EXTENSION_CLOSED_AUTO = -70;
    int EXTENSION_CAMERA = 600;
    int ROTATION_FRONT = 0;
    int ROTATION_BACK_HANG1 = 487; //508
    int ROTATION_LIFT = 487; //498 //was 467 before stopper
    int ROTATION_CHAMBER = 258;
    int ROTATION_PREASCEND = 110;
    int ROTATION_CAMERA = 130;

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
    public int offset = 0;
    public int rotOffset = 0;

    /* ------------------ CACHING ------------------ */
    double extPower = 0;
    double rotPower = 0;

    /* ------------------ PID ------------------ */
    /* no load 1:4 */ //public PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.0069, 0, 0.0003, -0.0064);
    /* no load 1:1.17 *///final PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.009, 0, 0.019, 0.028);
    /* load 1:1.17 *///final PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.02, 0, 0.04, 0.052);
    /* load 1:4 */ public PIDFCoefficients ROTATION_PIDF = new PIDFCoefficients(0.0023, 0, 0.0022, -0.0044);
    public PIDFCoefficients EXTENSION_PIDF = new PIDFCoefficients(0.0034, 0, 0.0034, 0.004);/*0.017*/ /*d: 0.00460 -> 0.00475*/
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
        YELLOW_1_PRO,
        YELLOW_2_PRO,
        YELLOW_3_PRO,
        YELLOW_AFKBOT,
        SUPPORT,
        PID_MANUAL,
        CLOSED_AUTO,
        DOBOR,
        CAMERA
    }

    public enum rotation {
        FRONT,
        BACK_HANG0,
        BACK_HANG1,
        CHAMBER,
        LIFT,
        RESET,
        MANUAL,
        PREASCEND,
        CAMERA
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
        //logger.writeLn("arm initialized");
    }

    public arm(HardwareMap HM, boolean teleop){
        rotationMotor = HM.get(DcMotorEx.class, "armRotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetRotationEncoders();

        extensionMotor = HM.get(DcMotorEx.class, "armExtensionMotor");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetExtensionEncoders();
        extensionState = extension.MANUAL;
        offset = transfer.armExtensionPos;
        this.teleop = teleop;
        if (teleop)
            rotOffset = 0;
        else {
            rotOffset = 487;
            offset = -70;
        }
        //logger.writeLn("offset: " + offset);

        rotationBtn = HM.get(AnalogInput.class, "rotationBtn");
        //logger.writeLn("arm initialized");
    }

    public void resetRotationEncoders() {
        //logger.writeLn("reset rotation encoder called");
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotOffset = 0;
    }

    public void resetExtensionEncoders() {
        //logger.writeLn("reset extension encoder called");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        offset = 0;
    }

    public void update(Telemetry telemetry) {
        telemetry.addLine("------------ARM------------");
        telemetry.addData("rot state", rotationState);
        telemetry.addData("rot pose", rotationMotor.getCurrentPosition() + rotOffset);
        telemetry.addData("wanted rot state", wantedRotation);
        telemetry.addData("voltage rot", rotationMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("target ext", targetExtensionPos);
        telemetry.addData("ext state", extensionState);
        telemetry.addData("voltage ext", extensionMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("ext pos", extensionMotor.getCurrentPosition() + offset);
        telemetry.addData("btn pressed", rotationBtn.getVoltage() < 0.4);
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
            telemetry.addData("rot pwr", setRotation(wantedRotation));

        if (extensionState.equals(extension.PID_MANUAL))
            pidExtend(targetExtensionPos);

        if (rotationMotor.getCurrentPosition() + rotOffset < 20 && rotationBtn.getVoltage() < 0.4 && (rotationState == rotation.FRONT || rotationState == rotation.MANUAL)) /* pressed */ {
            rotationState = rotation.RESET;
            resetRotationEncoders();
            setRotationMotorPower(0 + (teleop ? 0 : 1) * (-0.1));
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
                return (int)(-(132.57 / 928) * (rotationMotor.getCurrentPosition() + rotOffset));
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
            case YELLOW_1_PRO:
                return EXTENSION_YELLOW_1_PRO;
            case YELLOW_2_PRO:
                return EXTENSION_YELLOW_2_PRO;
            case YELLOW_3_PRO:
                return EXTENSION_YELLOW_3_PRO;
            case DOBOR:
                return EXTENSION_DOBOR;
            case YELLOW_AFKBOT:
                return EXTENSION_YELLOW_AFKBOT;
            case SUPPORT:
                return EXTENSION_SUPPORT;
            case CLOSED_AUTO:
                return EXTENSION_CLOSED_AUTO;
            case CAMERA:
                return EXTENSION_CAMERA;
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
            case PREASCEND:
                return ROTATION_PREASCEND;
            case FRONT:
                return ROTATION_FRONT;
            case CAMERA:
                return ROTATION_CAMERA;
            default:
                return -1;
        }
    }

    private double setExtensionMotorPower(double power){
        if (Math.abs(extPower - power) > 0.00001) {
            ////logger.writeLn("set extension motor power to " + power);
            this.extPower = power;
            this.extensionMotor.setPower(extPower);
        }
        return extPower;
    }

    private double setRotationMotorPower(double power){
        if (Math.abs(rotPower - power) > 0.001) {
            ////logger.writeLn("set rotation motor power to " + power);
            this.rotPower = power;
            this.rotationMotor.setPower(rotPower);
        }
        return rotPower;
    }

    private double pidCalculateExtensionPower(int targetPos){
        //if (extensionState == extension.EXTENDED && extensionMotor.getCurrentPosition() + offset < EXTENSION_FULL)
        //  return 1;
        int error = targetPos - extensionMotor.getCurrentPosition() - offset;
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
        double error = targetPos - rotationMotor.getCurrentPosition() - rotOffset;
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

        if (targetPos == 0 && error < 10)
            error *= (teleop ? 0.75 : 20);

        return (error * ROTATION_PIDF.p + delta * ROTATION_PIDF.d + ROTATION_PIDF.f * Math.sin(Math.toRadians(rotationAngle)) * (1 + ratio * ratio));
    }
    public double setExtension(extension target)
    {
        //logger.addData("target extension", target);
        this.targetExtensionPos = extensionPosToTicks(target);
        this.extensionState = target;

        if (Math.abs(rotationPosToTicks(rotationState) - rotationMotor.getCurrentPosition() - rotOffset) >= 120 && targetExtensionPos != EXTENSION_SUPPORT && targetExtensionPos != EXTENSION_CLOSED_AUTO && teleop)
            return this.setExtensionMotorPower(-0.1 + (teleop ? 0 : 1) * (-0.2));
        else if (rotationState != rotation.LIFT || rotationReached() || Math.abs(rotationPosToTicks(rotationState) - rotationMotor.getCurrentPosition() - rotOffset) < 420/* || target == extension.CLOSED*/)
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
        //logger.addData("wanted rotation", target);
        //logger.addData("real target rotation (on prev call)", rotationState);
        wantedRotation = target;

        if (wantedRotation == rotation.FRONT && rotationState == rotation.RESET)
            wantedRotation = rotation.RESET;

        if (wantedRotation == rotation.RESET && rotationState == rotation.RESET)
            return setRotationMotorPower(0 + (teleop ? 0 : 1) * (-0.1));

        if (rotationState == rotation.FRONT && wantedRotation == rotation.FRONT && rotationMotor.getCurrentPosition() + rotOffset < 20)
            return setRotationMotorPower(0);

        if (rotationState == rotation.RESET && rotationBtn.getVoltage() < 0.4 && rotationMotor.getCurrentPosition() + rotOffset < 100)
        {
            resetRotationEncoders();
        }

        if (target == rotation.PREASCEND || target == rotation.CHAMBER || target == rotation.BACK_HANG1 || rotationState == rotation.BACK_HANG1 || (extensionState == extension.CLOSED && extensionMotor.getCurrentPosition() + offset < 360) || (extensionMotor.getCurrentPosition() + offset < 240 && extensionState == extension.MANUAL) || (extensionState == extension.SUPPORT && extensionMotor.getCurrentPosition() + offset < 240) || (extensionState == extension.CLOSED_AUTO && extensionMotor.getCurrentPosition() + offset < 193) || (extensionState == extension.YELLOW_3_PRO && extensionMotor.getCurrentPosition() < EXTENSION_YELLOW_3_PRO + 86) || extensionState == extension.PID_MANUAL || extensionState == extension.CAMERA) {
            rotationState = wantedRotation;
            this.targetRotationPos = rotationPosToTicks(target);
        }

        if (target == rotation.LIFT && rotationMotor.getCurrentPosition() + rotOffset > ROTATION_LIFT - 20)
            return setRotationMotorPower(0.03 + (teleop ? 0 : 1) * 0.15);

        return this.setRotationMotorPower(pidCalculateRotationPower(targetRotationPos));
    }

    public void manuallyExtend(double speed){
        targetExtensionPos = extensionMotor.getCurrentPosition() + offset;
        if (rotationState == rotation.LIFT || this.extensionMotor.getCurrentPosition() + offset < extensionPosToTicks(extension.FRONTAL_EXTENSION) || speed < 0) {
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
        if (target < EXTENSION_FRONT_MAX)
            setExtensionMotorPower(pidCalculateExtensionPower(target));
    }

    public void manuallyRotate(double speed){
        this.rotationState = rotation.MANUAL;
        this.wantedRotation = rotation.MANUAL;
        setRotationMotorPower(speed);
    }

    public void stop(){
        //logger.writeLn("arm stopped");
        rotationState = rotation.MANUAL;
        extensionState = extension.MANUAL;
        this.extensionMotor.setPower(0);
        this.rotationMotor.setPower(0);
    }

    public double getRotationAngle(){
        double k = 180 / 928.0;
        double b = -90;

        return k * rotationMotor.getCurrentPosition() + rotOffset + b;
    }

    public double getExtensionLength(){
        double pos = extensionMotor.getCurrentPosition() + offset;
        double k = 0.715 / 1340;
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
        return (extensionMotor.getCurrentPosition() + offset >= EXTENSION_FULL - 955 && extensionState == extension.EXTENDED) ||
                (extensionMotor.getCurrentPosition() + offset < 23 && extensionState == extension.CLOSED) ||
                (Math.abs(extDelta) <= 5 && Math.abs(targetExtensionPos - extensionMotor.getCurrentPosition() - offset) < 15);
    }

    public boolean rotationReached()
    {
        return Math.abs(targetRotationPos - rotationMotor.getCurrentPosition() - rotOffset) < 15 && rotationState != rotation.FRONT || rotationState == rotation.LIFT && rotationMotor.getCurrentPosition() + rotOffset > ROTATION_LIFT /* && rotDeltaFiltered < 4*/;
    }

    /*
    static public int pixelToTicks(double px) {
        double a = 0.00135;
        double b = 1.23333;
        double c = 531.126;

        double result = a * px * px + b * px + c;

        return (int) Math.round(result);
    }
     */

}
