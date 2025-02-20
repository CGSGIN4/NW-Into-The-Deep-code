package org.firstinspires.ftc.teamcode.subsystems.modules;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RR.util.Encoder;
import org.firstinspires.ftc.teamcode.utils.logger;

@Config
public class hang {
    public hang.states state = states.SLEEPING;
    DcMotorEx left;
    DcMotorEx right;
    ElapsedTime timer = new ElapsedTime();

    public enum states{
        SLEEPING,
        PREPARE,
        PREPARE3,
        READY,
        ASCEND2COMPLETE,
        READY3, FOLDING, ASCEND3COMPLETE, FOLDING3,
        STANDBY3
    }
    PIDCoefficients pid = new PIDCoefficients(0.003, 0, 0.001);
    public static int target_2_extended = 1900;
    public static int target_3_extended = 2915;
    public static int target_2_hang = -100;
    int oldErrLeft = 0;
    int oldErrRight = 0;
    IMU imu;
    public static double startPitch = 0;
    public static double perhvat_angle = -2;
    public static double third_angle = 16;

    public hang(HardwareMap HM){
        left = HM.get(DcMotorEx.class, "hangLeft");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setDirection(DcMotor.Direction.REVERSE);

        right = HM.get(DcMotorEx.class, "hangRight");
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = HM.get(IMU.class, "imu");
        //logger.writeLn("hang initialized");
    }

    private void goTo(int target)
    {
        int lError = target - left.getCurrentPosition();
        int rError = target - right.getCurrentPosition();

        int lDelta = lError - oldErrLeft;
        int rDelta = rError - oldErrRight;
        if (state == states.FOLDING3)
        {
            left.setPower(-1);
            right.setPower(-1);
        }
        else {
            left.setPower(lError * pid.p + lDelta * pid.d);
            right.setPower(rError * pid.p + rDelta * pid.d);
        }
        oldErrLeft = lError;
        oldErrRight = rError;
    }

    public void prepare(){
        state = states.PREPARE;
        goTo(target_2_extended);
    }

    public void prepare3(){
        state = states.PREPARE3;
        goTo(target_3_extended);
    }

    public void fold(){
        state = states.FOLDING;
        goTo(target_2_hang);
    }

    public void pushDown(){
        goTo(target_2_hang);
    }
    public void pushDownGently(){
        left.setPower(-0.4);
        right.setPower(-0.4);
    }
    public void pushUp(){
        goTo(target_3_extended);
    }

    public void fold3(){
        state = states.FOLDING3;
        goTo(target_2_hang);
    }

    public void chill(){
        left.setPower(0);
        right.setPower(0);
    }
    public void standby3(){
        state = states.STANDBY3;
        goTo(2540);
    }
    public void setPower(double power){
        left.setPower(power);
        right.setPower(power);
    }

    public void update(){
        update(new MultipleTelemetry());
    }
    public void update(MultipleTelemetry telemetry){
        telemetry.addLine("------------HANG------------");
        telemetry.addData("hang state", state.toString());
        telemetry.addData("left pos", left.getCurrentPosition());
        telemetry.addData("right pos", right.getCurrentPosition());
        telemetry.addData("left voltage", left.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("right voltage", right.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("left pwr", left.getPower());
        telemetry.addData("right pwr", right.getPower());
        telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        switch (state)
        {
            case PREPARE:
                startPitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
                if (Math.abs(left.getCurrentPosition() - target_2_extended) < 20 && Math.abs(right.getCurrentPosition() - target_2_extended) < 20) {
                    state = states.READY;
                }
                else
                    prepare();
                break;
            case READY:
                fold();
                break;
            case FOLDING:
                if (Math.abs(left.getCurrentPosition() - target_2_hang) < 20 && Math.abs(right.getCurrentPosition() - target_2_hang) < 20) {
                    state = states.ASCEND2COMPLETE;
                }
                else
                    fold();
                break;
            case ASCEND2COMPLETE:
                pushDown();
                if (imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) - startPitch < perhvat_angle)
                    prepare3();
                break;
            case PREPARE3:
                if (Math.abs(left.getCurrentPosition() - target_3_extended) < 20 && Math.abs(right.getCurrentPosition() - target_3_extended) < 20)
                    state = states.READY3;
                else
                    pushUp();
                break;
            case READY3:
                //fold3();
                pushUp();
                if (imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) - startPitch > third_angle)
                    state = states.STANDBY3;
                break;
            case STANDBY3:
                standby3();
                break;
            case FOLDING3:
                if (Math.abs(left.getCurrentPosition() - target_2_hang) < 50 && Math.abs(right.getCurrentPosition() - target_2_hang) < 50)
                    state = states.ASCEND3COMPLETE;
                else
                    fold3();
                break;
            case ASCEND3COMPLETE:
                pushDownGently();
                break;
        }
    }
}
