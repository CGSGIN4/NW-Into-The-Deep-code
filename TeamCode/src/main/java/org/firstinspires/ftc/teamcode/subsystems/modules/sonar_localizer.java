package org.firstinspires.ftc.teamcode.subsystems.modules;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.sensors.MB1242;
import org.firstinspires.ftc.teamcode.sensors.sonar;
import org.firstinspires.ftc.teamcode.utils.wall;

public class sonar_localizer {
    HardwareMap hardwareMap;
    sonar sensorFront;
    sonar sensorBack;
    sonar sensorRight;
    sonar sensorLeft;
    double distFront = 0;
    double distBack = 0;
    double distRight = 0;
    double distLeft = 0;


    /**
     *
     * @deprecated constructor assumes there are 4 MB1242 sonars
     */
    public sonar_localizer(HardwareMap HM){
        this.hardwareMap = HM;
        this.sensorFront = new sonar(hardwareMap.get(MB1242.class, "sensorFront"), new Vector2d(0, 0), 0, sonar.sonar_type.MB1242);
        this.sensorBack = new sonar(hardwareMap.get(MB1242.class, "sensorBack"), new Vector2d(0, 0), Math.PI, sonar.sonar_type.TWO_EYED);
        this.sensorRight = new sonar(hardwareMap.get(MB1242.class, "sensorRight"), new Vector2d(0, 0), -Math.PI / 2, sonar.sonar_type.TWO_EYED);
        this.sensorLeft = new sonar(hardwareMap.get(MB1242.class, "sensorLeft"), new Vector2d(0, 0), Math.PI / 2, sonar.sonar_type.OTHER);
    }

    /**
     * Default class constructor
     */
    public sonar_localizer(sonar sensorFront, sonar sensorLeft, sonar sensorBack, sonar sensorRight){
        this.sensorFront = sensorFront;
        this.sensorLeft = sensorLeft;
        this.sensorRight = sensorRight;
        this.sensorBack = sensorBack;
    }

    /**
     * Pings all MB1242 sensors
     */
    public void ping(){
        sensorFront.ping();
        sensorBack.ping();
        sensorRight.ping();
        sensorLeft.ping();
    }

    /**
     * @return estimated robot center position using sonars.
     * @Use: SampleMecanumDrive.setPoseEstimate(sonar_localizer.getPosition());
     */
    public Vector2d getPosition(){
        Vector2d[] readings = new Vector2d[] {sensorFront.getCoordinate(),
                                              sensorLeft.getCoordinate(),
                                              sensorBack.getCoordinate(),
                                              sensorRight.getCoordinate()};
        double[] xCoords = new double[] {1000, 1000, 1000, 1000};
        double[] yCoords = new double[] {1000, 1000, 1000, 1000};

        for (int i = 0; i < 4; i++){
            dataStorage.DSTelemetry.addData(i + "", readings[i].toString());
            if (Math.abs(readings[i].getX() - dataStorage.RobotWorldX) < 30)
                xCoords[i] = readings[i].getX();
            if (Math.abs(readings[i].getY() - dataStorage.RobotWorldY) < 30)
                yCoords[i] = readings[i].getY();
        }

        int xCnt = 0, yCnt = 0;
        double xSum = 0, ySum = 0;
        for (int i = 0; i < 4; i++){
            if (xCoords[i] != 1000) {
                xCnt++;
                xSum += xCoords[i];
            }
            if (yCoords[i] != 1000) {
                yCnt++;
                ySum += yCoords[i];
            }
        }

        double estimatedX = dataStorage.RobotWorldX;
        double estimatedY = dataStorage.RobotWorldY;
        if (xCnt != 0)
            estimatedX = xSum / xCnt;
        if (yCnt != 0)
            estimatedY = ySum / yCnt;

        dataStorage.DSTelemetry.addData("real pos", dataStorage.RobotPose);
        dataStorage.DSTelemetry.addData("updated X", estimatedX != dataStorage.RobotWorldX);
        dataStorage.DSTelemetry.addData("updated Y", estimatedY != dataStorage.RobotWorldY);
        return new Vector2d(estimatedX, estimatedY);
    }
}
