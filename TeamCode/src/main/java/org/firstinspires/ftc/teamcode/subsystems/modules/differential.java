package org.firstinspires.ftc.teamcode.subsystems.modules;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public differential(HardwareMap HM){
        lServo = HM.get(Servo.class, "differential_l");
        rServo = HM.get(Servo.class, "differential_r");
        claw = HM.get(Servo.class, "claw");
        pitch = 180;
        roll = 0;
    }
    double posToDeg(double pos){
        return MAX_ANGLE * pos;
    }

    double angleToPos(double angle){
        return angle / MAX_ANGLE;
    }

    public void setPitch(double angle)
    {
        angle = max(20, min(angle, 180));
        pitch = angle + 80;
    }

    public void setRoll(double angle)
    {
        roll = max(-80, (min(angle, 80)));
    }

    public void pitchUp()
    {
        setPitch(180);
    }

    public void pitchDown()
    {
        setPitch(20);
    }

    public void pitchForward()
    {
        setPitch(100);
    }

    public void rollFullLeft()
    {
        setRoll(-80);
    }

    public void rollHalfLeft()
    {
        setRoll(-45);
    }

    public void rollHalfRight()
    {
        setRoll(45);
    }

    public void rollFullRight()
    {
        setRoll(80);
    }

    public void rollDefault()
    {
        setRoll(2);
    }

    public void openClaw(){
        claw.setPosition(0.6);
    }

    public void closeClaw(){
        claw.setPosition(0.32);
    }

    public void update(){
        lTheta = pitch + roll;
        rTheta = pitch - roll;
        lServo.setPosition(1 - angleToPos(lTheta));
        rServo.setPosition(angleToPos(rTheta));
    }
}
