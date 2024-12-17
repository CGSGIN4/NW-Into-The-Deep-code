package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class painter {
    FtcDashboard dashboard;
    TelemetryPacket packet;
    Canvas canvas;
    public painter() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    public void prepare(TelemetryPacket packet, Canvas canvas){
        this.packet = packet;
        this.canvas = canvas;
    }

    public void drawPolyLine(Vector2d[] points, String color){
        double[] pointsX = new double[points.length];
        double[] pointsY = new double[points.length];

        for (int i = 0; i < points.length; i++)
        {
            pointsX[i] = points[i].getX();
            pointsY[i] = points[i].getY();
        }

        canvas.setStroke(color);
        canvas.strokePolyline(pointsX, pointsY);
    }

    public void drawVector(Vector2d start, Vector2d end, String color){
        canvas.setStroke(color);
        canvas.strokeLine(start.getX(), start.getY(), end.getX(), end.getY());
    }

    public void drawVector(Vector2d start, Vector2d end){
        drawVector(start, end, "green");
    }
    public void drawPolyLine(Vector2d[] points){
        drawPolyLine(points, "green");
    }

    public void drawPoint(double X, double Y, String color){
        canvas.setFill(color);
        canvas.fillCircle(X, Y, 2);
    }

    public void drawGround(double X, double Y, String color){
        canvas.setFill(color);
        canvas.fillCircle(X, Y, 3);
    }

    //15.339 длинная в дюймах
    //12.677 короткая в дюймах
    public void drawRobot(double x, double y, double rotation, String color){
        canvas.setStroke(color);
        double len = 16.2;
        double wid = 14.133;
        rotation = Math.PI / 2 - rotation;
        double x1 = x + len / 2 * Math.sin(rotation);
        double y1 = y + len / 2 * Math.cos(rotation);
        double x3 = x1 - wid / 2 * Math.cos(rotation);
        double y3 = y1 + wid / 2 * Math.sin(rotation);
        double x4 = x3 - len * Math.sin(rotation);
        double y4 = y3 - len * Math.cos(rotation);
        double x5 = x4 + wid * Math.cos(rotation);
        double y5 = y4 - wid * Math.sin(rotation);
        double x6 = x5 + len * Math.sin(rotation);
        double y6 = y5 + len * Math.cos(rotation);
        canvas.strokeLine(x4, y4, x5, y5);
        canvas.strokeLine(x4, y4, x3, y3);
        canvas.strokeLine(x3, y3, x6, y6);
        canvas.strokeLine(x5, y5, x6, y6);
    }
    public void drawPoint(double X, double Y){
        drawPoint(X, Y, "green");
    }
}
