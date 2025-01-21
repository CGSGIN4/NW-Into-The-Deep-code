package org.firstinspires.ftc.teamcode.sensors;

import static org.firstinspires.ftc.teamcode.math.normalizeAngle.normalizeAngle;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.calculator;
import org.firstinspires.ftc.teamcode.utils.wall;

public class sonar {
    AnalogInput sonar = null;
    MB1242 mbSonar = null;
    public Vector2d OFFSET = new Vector2d();
    public double ANGULAR_OFFSET;
    public wall wall;
    private sonar_type type;

    public enum sonar_type {
        MB1242,
        TWO_EYED,
        TWO_EYED_LEFT,
        OTHER
    }

    public sonar(AnalogInput sonar, Vector2d offset, double angOffset, sonar_type type){
        this.type = type;
        this.sonar = sonar;
        this.OFFSET = offset;
        this.ANGULAR_OFFSET = angOffset;
    }

    public sonar(MB1242 sonar, Vector2d offset, double angOffset, sonar_type type){
        this.type = type;
        this.mbSonar = sonar;
        this.OFFSET = offset;
        this.ANGULAR_OFFSET = angOffset;
    }

    public void ping(){
        if (type == sonar_type.MB1242)
            this.mbSonar.ping();
    }

    /** Returns sensor readings in MM */
    private double getDistance(){
        double x, x2, x3;
        switch (type){
            case TWO_EYED:
                x = sonar.getVoltage();
                x2 = x * x;
                x3 = x2 * x;
                return 693.0241912156343 * x3 - 938.2597099989653 * x2 + 2586.2266138717532 * x - 350.6372525286861;
            case TWO_EYED_LEFT:
                x = sonar.getVoltage();
                x2 = x * x;
                x3 = x2 * x;
                return -2241.31085801125 * x3 + 2252.79726028442 * x2 + 1534.79490369558 * x - 235.84690043330;
            case OTHER:
                x = sonar.getVoltage();
                x2 = x * x;
                x3 = x2 * x;
                return 383368.0672441720963 * x3 - 88995.0556239187717 * x2 + 13651.3481714744121 * x - 126.7881081727683;
            case MB1242:
                return mbSonar.getDistance(DistanceUnit.MM);
        }
        return 0;
    }
    private Vector2d getFieldCoords(){
        return dataStorage.RobotPose.plus(OFFSET.rotated(dataStorage.RobotWorldHeading));
    }
    /** I'm sorry for this implementation, but idk how to do it better.
     * calls getFieldCoords() */
    private void findWall(){
        Vector2d selfPose = getFieldCoords();
        double angle = normalizeAngle(dataStorage.RobotWorldHeading + ANGULAR_OFFSET);

        double t;
        while (true){
            t = calculator.findRaySegmentIntersectionAndGetT(selfPose, angle, new Vector2d(-72, 72), new Vector2d(72, 72));
            if (t > 0.2 && t < 0.8){
                this.wall = org.firstinspires.ftc.teamcode.utils.wall.BLUE;
                break;
            }
            else if (t != -1)
            {
                this.wall = org.firstinspires.ftc.teamcode.utils.wall.NONE;
                break;
            }

            t = calculator.findRaySegmentIntersectionAndGetT(selfPose, angle, new Vector2d(72, -72), new Vector2d(72, 72));
            if (t > 0.2 && t < 0.8){
                this.wall = org.firstinspires.ftc.teamcode.utils.wall.JUDGE;
                break;
            }
            else if (t != -1)
            {
                this.wall = org.firstinspires.ftc.teamcode.utils.wall.NONE;
                break;
            }

            t = calculator.findRaySegmentIntersectionAndGetT(selfPose, angle, new Vector2d(72, -72), new Vector2d(-72, -72));
            if (t > 0.2 && t < 0.8){
                this.wall = org.firstinspires.ftc.teamcode.utils.wall.RED;
                break;
            }
            else if (t != -1)
            {
                this.wall = org.firstinspires.ftc.teamcode.utils.wall.NONE;
                break;
            }

            t = calculator.findRaySegmentIntersectionAndGetT(selfPose, angle, new Vector2d(-72, 72), new Vector2d(-72, -72));
            if (t > 0.2 && t < 0.8){
                this.wall = org.firstinspires.ftc.teamcode.utils.wall.AUDIENCE;
                break;
            }
            else if (t != -1)
            {
                this.wall = org.firstinspires.ftc.teamcode.utils.wall.NONE;
                break;
            }
        }
    }

    public Vector2d getCoordinate(){
        double angle = normalizeAngle(dataStorage.RobotWorldHeading + ANGULAR_OFFSET);
        findWall();

        dataStorage.DSTelemetry.addData(this.OFFSET.toString(), this.getDistance());
        if (wall == org.firstinspires.ftc.teamcode.utils.wall.BLUE)
            return new Vector2d(1000, 72 - getDistance() / 25.4 * Math.sin(angle)).minus(OFFSET.rotated(dataStorage.RobotWorldHeading));
        else if (wall == org.firstinspires.ftc.teamcode.utils.wall.JUDGE)
            return new Vector2d(72 - getDistance() / 25.4 * Math.cos(angle), 1000).minus(OFFSET.rotated(dataStorage.RobotWorldHeading));
        else if (wall == org.firstinspires.ftc.teamcode.utils.wall.RED)
            return new Vector2d(1000, -72 - getDistance() / 25.4 * Math.sin(angle)).minus(OFFSET.rotated(dataStorage.RobotWorldHeading));
        else if (wall == org.firstinspires.ftc.teamcode.utils.wall.AUDIENCE)
            return new Vector2d(-72 - getDistance() / 25.4 * Math.cos(angle), 1000).minus(OFFSET.rotated(dataStorage.RobotWorldHeading));
        else
            return new Vector2d(1000, 1000);
    }
}
