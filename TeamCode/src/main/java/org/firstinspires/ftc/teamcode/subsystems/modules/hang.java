package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class hang {
    DcMotor left;
    DcMotor right;

    PIDCoefficients pid = new PIDCoefficients();
    public hang(HardwareMap HM){
        left = HM.get(DcMotor.class, "hangLeft");
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right = HM.get(DcMotor.class, "hangRight");
    }

    public void hold(int ticks)
    {
        int lError = left.getCurrentPosition();
    }
}
