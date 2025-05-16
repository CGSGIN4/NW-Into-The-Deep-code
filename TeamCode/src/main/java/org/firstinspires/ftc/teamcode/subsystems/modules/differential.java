package org.firstinspires.ftc.teamcode.subsystems.modules;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.logger;

public class differential {
    /* ------------------ HARDWARE ------------------ */
    public Servo rServo;
    public Servo lServo;
    public Servo claw;

    /* ------------------ CONSTANTS ------------------ */
    int MAX_ANGLE = 340; /* max servo angle */

    /* ------------------ ON-FLY ------------------ */
    public double lTheta;
    public double rTheta;
    public double pitch; /* up/down */
    public double roll; /* rotation */
    public int clawState = 0; /* 1 - open; 0 - close */

    public differential(HardwareMap HM){
        lServo = HM.get(Servo.class, "differential_l");
        rServo = HM.get(Servo.class, "differential_r");
        claw = HM.get(Servo.class, "claw");
        pitch = 180;
        roll = 0;
        ////logger.writeLn("differential initialized");
    }
    double posToDeg(double pos){
        return MAX_ANGLE * pos;
    }

    double angleToPos(double angle){
        return angle / MAX_ANGLE;
    }

    public double setPitch(double angle)
    {
        angle = max(6, min(angle, 176));
        pitch = angle + 80;
        return angle;
    }

    public double setRoll(double angle)
    {
        roll = max(-80, (min(angle, 75)));
        return angle;
    }

    public void pitchUp()
    {
        setPitch(168);
        update();
    }

    public void pitchDown()
    {
        setPitch(6);
        update();
    }

    public void pitchHalfDown()
    {
        setPitch(15);
        update();
    }

    public void pitchScoringBasket()
    {
        setPitch(150);
        update();
    }

    public void pitchScoringBasketDobor()
    {
        setPitch(120);
        update();
    }

    public void pitchForward()
    {
        setPitch(99);
        update();
    }

    public void rollFullLeft()
    {
        setRoll(-80);
        update();
    }

    public void rollHalfLeft()
    {
        setRoll(-45);
        update();
    }

    public void rollHalfRight()
    {
        setRoll(45);
        update();
    }

    public void rollFullRight()
    {
        setRoll(75);
        update();
    }

    public void rollDefault()
    {
        setRoll(-11);
        update();
    }

    public void openClaw(){
        clawState = 1;
        claw.setPosition(0.95);
    }

    public void closeClaw(){
        clawState = 0;
        claw.setPosition(0.56);
    }

    public void closeClawSilno(){
        clawState = 0;
        claw.setPosition(0.55);
    }

    public void closeClawVerySilno(){
        clawState = 0;
        claw.setPosition(0.535);
    }

    public void clawSwitch(){
        if (clawState == 0)
            openClaw();
        else
            closeClaw();
    }

    // convert angle from [0; 180] to [-80; 80]
    public static double geomToDifAngle(double angle) {
        return -80 + (angle / 180.0) * 160;
    }

    public void update(){
        lTheta = pitch + roll;
        rTheta = pitch - roll;
        lServo.setPosition(1 - angleToPos(lTheta));
        rServo.setPosition(angleToPos(rTheta));
    }

    public void update(MultipleTelemetry telemetry){
        lTheta = pitch + roll;
        rTheta = pitch - roll;
        lServo.setPosition(1 - angleToPos(lTheta));
        rServo.setPosition(angleToPos(rTheta));

        telemetry.addLine("------------DIFFY------------");
        telemetry.addData("pitch", pitch);
        telemetry.addData("roll", roll);
    }
}
