package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class curve {
    int DEFAULT_ITER = 20;
    public Vector2d[] points;
    public Vector2d[] nodes = new Vector2d[4];
    public ArrayList<Double> turn_starts = new ArrayList<>();
    public ArrayList<Double> turn_ends = new ArrayList<>();

    public curve(Vector2d[] points, Vector2d[] nodes) throws Exception {
        if (nodes.length != 4)
            throw new Exception(String.format("illegal nodes amount: must be 4, given %d, last node %f, %f", nodes.length, nodes[nodes.length - 1].getX(), nodes[nodes.length - 1].getY()));

        this.points = points.clone();
        this.nodes = nodes.clone();
    }
    public curve(Vector2d[] nodes, int iter) throws Exception {
        if (nodes.length != 4)
            throw new Exception(String.format("illegal nodes amount: must be 4, given %d, last node %f, %f", nodes.length, nodes[nodes.length - 1].getX(), nodes[nodes.length - 1].getY()));

        this.nodes = nodes.clone();
        this.points = bezierCurves.buildCubicBernstein(nodes, iter);
    }
    public curve(Vector2d[] nodes) throws Exception {
        if (nodes.length != 4)
            throw new Exception(String.format("illegal nodes amount: must be 4, given %d, last node %f, %f", nodes.length, nodes[nodes.length - 1].getX(), nodes[nodes.length - 1].getY()));

        this.nodes = nodes.clone();
        this.points = bezierCurves.buildCubicBernstein(nodes, DEFAULT_ITER);
    }

    public Vector2d getPoint(double t){
        Vector2d point;

        double t2 = t * t;
        double t3 = t2 * t;

        Vector2d p1 = this.nodes[0];
        Vector2d p2 = this.nodes[1];
        Vector2d p3 = this.nodes[2];
        Vector2d p4 = this.nodes[3];

        point = p1.times(-t3 + 3*t2 - 3*t + 1).plus(p2.times(3*t3 - 6*t2 + 3*t)).plus(p3.times(-3*t3 + 3*t2)).plus(p4.times(t3));
        return point;
    }

    public Vector2d getFirstDerivative(double t){
        Vector2d velocity;

        double t2 = t * t;
        Vector2d p1 = this.nodes[0];
        Vector2d p2 = this.nodes[1];
        Vector2d p3 = this.nodes[2];
        Vector2d p4 = this.nodes[3];

        velocity = p1.times(-3*t2 + 6*t - 3).plus(p2.times(9*t2 - 12*t + 3)).plus(p3.times(-9*t2 + 6*t)).plus(p4.times(3*t2));
        return velocity;
    }

    public Vector2d getSecondDerivative(double t){
        Vector2d accel;

        Vector2d p1 = this.nodes[0];
        Vector2d p2 = this.nodes[1];
        Vector2d p3 = this.nodes[2];
        Vector2d p4 = this.nodes[3];

        accel = p1.times(-6*t + 6).plus(p2.times(18*t - 12)).plus(p3.times(-18*t + 6)).plus(p4.times(6*t));
        return accel;
    }

    public double getRadius(double t){

        Vector2d accel = getSecondDerivative(t);
        double velocityModule = getFirstDerivative(t).norm();

        return velocityModule * velocityModule / accel.norm();
    }

}
