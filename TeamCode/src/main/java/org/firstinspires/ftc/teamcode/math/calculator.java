package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.data.dataStorage;

public enum calculator {;
    public static Vector2d findRaySegmentIntersection(Vector2d rayOrigin, double vectorAngle, Vector2d segmentStart, Vector2d segmentEnd){
        Vector2d delta = new Vector2d(0, 0.1).rotated(vectorAngle);
        //dataStorage.DSTelemetry.addData("delta", delta.toString());
        //dataStorage.DSTelemetry.update();
        if ((-segmentEnd.getX() * delta.getY() + segmentEnd.getY() * delta.getX() - delta.getX() * segmentStart.getY() + segmentStart.getX() * delta.getY()) == 0)
            return null;
        if (segmentStart.getX() == segmentEnd.getX() && segmentStart.getY() == segmentEnd.getY())
            return null;
        //dataStorage.DSTelemetry.addData("1st condition passed", "true");
        //dataStorage.DSTelemetry.update();
        double tRay = (-rayOrigin.getX() * segmentEnd.  getY() + rayOrigin.getX() * segmentStart.getY() + rayOrigin.getY() * segmentEnd.getX() - rayOrigin.getY() * segmentStart.getX() - segmentEnd.getX() * segmentStart.getY() + segmentEnd.getY() * segmentStart.getX()) / (-segmentEnd.getX() * delta.getY() + segmentEnd.getY() * delta.getX() - delta.getX() * segmentStart.getY() + segmentStart.getX() * delta.getY());
        if (tRay < 0)
            return null;
        //dataStorage.DSTelemetry.addData("2nd condition passed", "true");
        //dataStorage.DSTelemetry.update();
        double tSegment;
        if (segmentEnd.getX() != segmentStart.getX())
            tSegment = (rayOrigin.getX() + delta.getX() * tRay - segmentStart.getX()) / (segmentEnd.getX() - segmentStart.getX());
        else
            tSegment = (rayOrigin.getY() + delta.getY() * tRay - segmentStart.getY()) / (segmentEnd.getY() - segmentStart.getY());
        if (tSegment < 0 || tSegment > 1)
            return null;
        //dataStorage.DSTelemetry.addData("3rd condition passed", "true");
        //dataStorage.DSTelemetry.update();
        return (new Vector2d(rayOrigin.getX() + delta.getX() * tRay, rayOrigin.getY() + delta.getY() * tRay));
    }

    public static double findRaySegmentIntersectionAndGetT(Vector2d rayOrigin, double vectorAngle, Vector2d segmentStart, Vector2d segmentEnd){
        Vector2d delta = new Vector2d(0.1, 0).rotated(vectorAngle);
        //dataStorage.DSTelemetry.addData("delta", delta.toString());
        //dataStorage.DSTelemetry.update();
        if ((-segmentEnd.getX() * delta.getY() + segmentEnd.getY() * delta.getX() - delta.getX() * segmentStart.getY() + segmentStart.getX() * delta.getY()) == 0)
            return -1;
        if (segmentStart.getX() == segmentEnd.getX() && segmentStart.getY() == segmentEnd.getY())
            return -1;
        //dataStorage.DSTelemetry.addData("1st condition passed", "true");
        //dataStorage.DSTelemetry.update();
        double tRay = (-rayOrigin.getX() * segmentEnd.getY() + rayOrigin.getX() * segmentStart.getY() + rayOrigin.getY() * segmentEnd.getX() - rayOrigin.getY() * segmentStart.getX() - segmentEnd.getX() * segmentStart.getY() + segmentEnd.getY() * segmentStart.getX()) / (-segmentEnd.getX() * delta.getY() + segmentEnd.getY() * delta.getX() - delta.getX() * segmentStart.getY() + segmentStart.getX() * delta.getY());
        if (tRay < 0)
            return -1;
        //dataStorage.DSTelemetry.addData("2nd condition passed", "true");
        //dataStorage.DSTelemetry.update();
        double tSegment;
        if (segmentEnd.getX() != segmentStart.getX())
            tSegment = (rayOrigin.getX() + delta.getX() * tRay - segmentStart.getX()) / (segmentEnd.getX() - segmentStart.getX());
        else
            tSegment = (rayOrigin.getY() + delta.getY() * tRay - segmentStart.getY()) / (segmentEnd.getY() - segmentStart.getY());
        if (tSegment < 0 || tSegment > 1)
            return -1;
        return tSegment;
    }
}
