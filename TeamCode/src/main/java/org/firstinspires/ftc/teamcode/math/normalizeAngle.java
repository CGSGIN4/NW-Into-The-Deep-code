package org.firstinspires.ftc.teamcode.math;

import static java.lang.Math.PI;

public class normalizeAngle{
    public static double normalizeAngle(double angle){
        if (angle > PI)
            angle = angle - 2 * PI;
        else if (angle < -PI)
            angle = 2 * PI + angle;

        return angle;
    }
}
