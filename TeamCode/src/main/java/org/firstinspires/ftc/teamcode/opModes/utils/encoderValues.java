package org.firstinspires.ftc.teamcode.opModes.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Telemetry Encoder Values", group = "Utils")
public class encoderValues extends OpMode {

    // Моторы
    private DcMotor motorLR;
    private DcMotor motorLF;
    private DcMotor motorRR;
    private DcMotor motorRF;
    private DcMotor armExtensionMotor;
    private DcMotor armRotationMotor;
    private DcMotor hangLeft;
    private DcMotor hangRight;

    @Override
    public void init() {
        // Инициализация моторов
        motorLR = hardwareMap.get(DcMotor.class, "MotorLR");
        motorLF = hardwareMap.get(DcMotor.class, "MotorLF");
        motorRR = hardwareMap.get(DcMotor.class, "MotorRR");
        motorRF = hardwareMap.get(DcMotor.class, "MotorRF");
        armExtensionMotor = hardwareMap.get(DcMotor.class, "armExtensionMotor");
        armRotationMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
        hangLeft = hardwareMap.get(DcMotor.class, "hangLeft");
        hangRight = hardwareMap.get(DcMotor.class, "hangRight");

        // Убедимся, что энкодеры работают
        motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Вывод в телеметрию значений энкодеров
        telemetry.addData("MotorLR Encoder", motorLR.getCurrentPosition());
        telemetry.addData("MotorLF Encoder", motorLF.getCurrentPosition());
        telemetry.addData("MotorRR Encoder", motorRR.getCurrentPosition());
        telemetry.addData("MotorRF Encoder", motorRF.getCurrentPosition());
        telemetry.addData("Arm Extension Encoder", armExtensionMotor.getCurrentPosition());
        telemetry.addData("Arm Rotation Encoder", armRotationMotor.getCurrentPosition());
        telemetry.addData("Hang Left Encoder", hangLeft.getCurrentPosition());
        telemetry.addData("Hang Right Encoder", hangRight.getCurrentPosition());
        telemetry.update();
    }
}

