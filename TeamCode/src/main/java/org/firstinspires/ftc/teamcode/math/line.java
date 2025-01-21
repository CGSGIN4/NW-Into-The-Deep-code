package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class line {
    double x1;
    double y1;
    double x2;
    double y2;

    double k;
    double b;

    public line(double x1, double x2, double y1, double y2){
        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;
        writeKB();
    }

    public line(double k, double b){
        this.k = k;
        this.b = b;
        writePoints();
    }

    private void writePoints(){
        this.x1 = -80;
        this.y1 = k * x1 + b;
        this.x2 = 80;
        this.y2 = k * x2 + b;
    }
    private void writeKB(){
        this.k = (y1 - y2) / (x1 - x2);
        this.b = y1 - k * x1;
    }

    public line MiddlePerpendicular(){
        Vector2d center = new Vector2d((x1 + x2) / 2, (y1 + y2) / 2);
        double k2 = -1 / k;
        double b2 = center.getY() - k2 * center.getX();

        return new line(k2, b2);
    }

    public Vector2d intersection(line other) throws ArithmeticException {
        if (k == other.k)
            throw new ArithmeticException("parallel");
        double x = (other.b - this.b) / (this.k - other.k);
        double y = k * x + b;

        return new Vector2d(x, y);
    }
}
