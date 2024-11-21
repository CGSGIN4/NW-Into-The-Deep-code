package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class differential {
    /* ------------------ HARDWARE ------------------ */
    public Servo rServo;
    public Servo lServo;

    /* ------------------ CONSTANTS ------------------ */
    int MAX_ANGLE = 180; /* max servo angle */

    /* ------------------ ON-FLY ------------------ */
    public double lTheta;
    public double rTheta;
    public double pitch; /* up/down */
    public double roll; /* rotation */

    public differential(HardwareMap HM){
        lServo = HM.get(Servo.class, "differential_l");
        rServo = HM.get(Servo.class, "differential_r");
        lTheta = posToDeg(lServo.getPosition());
        rTheta = posToDeg(rServo.getPosition());
        roll = lTheta - rTheta;
        pitch = lTheta - roll / 2;
    }
    double posToDeg(double pos){
        return MAX_ANGLE * pos;
    }

    double angleToPos(double angle){
        return angle / MAX_ANGLE;
    }

    public void setPitch(double angle)
    {
        pitch = angle;
    }

    public void setRoll(double angle)
    {
        roll = angle;
    }

    public void update(){
        lTheta = pitch + (roll / 2);
        rTheta = pitch - (roll / 2);
        lServo.setPosition(angleToPos(lTheta));
        rServo.setPosition(angleToPos(rTheta));
    }
}
