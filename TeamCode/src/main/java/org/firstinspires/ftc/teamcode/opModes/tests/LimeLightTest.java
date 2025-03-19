package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;

import java.util.List;

@TeleOp
@Config
public class LimeLightTest extends LinearOpMode {
    private Limelight3A ll;
    public static double tolerance = 0.8;
    @Override
    public void runOpMode() throws InterruptedException {

        differential diffy = new differential(hardwareMap);
        diffy.setPitch(0);
        ll = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(200);

        ll.setPollRateHz(10);

        ll.pipelineSwitch(0);
        ll.start();

        waitForStart();

        double lastAng = 90;
        while (opModeIsActive()) {
            double differentSides = 0;
            LLResult result = ll.getLatestResult();
            if (result != null && result.isValid()) {
                LLResultTypes.ColorResult colorResult = result.getColorResults().get(0);

                double xDeg = colorResult.getTargetXDegrees();
                double yDeg = colorResult.getTargetYDegrees();
                double area = colorResult.getTargetArea();

                // Создаём массив углов
                List<Double>[] corners = new List[colorResult.getTargetCorners().size()];
                double[] dists = new double[colorResult.getTargetCorners().size()];

                for (int i = 0; i < colorResult.getTargetCorners().size(); i++)
                {
                    corners[i] = colorResult.getTargetCorners().get(i);
                }

                double minRatio = 100;
                double Ratio = 0;
                double maxDistance = 0;
                List<Double> farthest1 = null;
                List<Double> farthest2 = null;
                double lastDistance = -1;
                boolean allEqual = true; // Флаг равенства расстояний

                double tg, ang;
                if (corners.length == 4) {
                    // Проверяем расстояние только между последовательными точками
                    for (int i = 1; i < colorResult.getTargetCorners().size(); i++) {
                        double dist = calculateDistance(corners[i], corners[i - 1]);
                        dists[i] = dist;
                        if ((Ratio = Math.min(dist, lastDistance) / Math.max(dist, lastDistance)) < Math.abs(tolerance) && Ratio > 0) {
                            allEqual = false;
                        }

                        if (Ratio < minRatio && Ratio > 0 && Ratio < 1)
                            minRatio = Ratio;
                        
                        lastDistance = dist;
                        if (dist > maxDistance) {
                            maxDistance = dist;
                            farthest1 = corners[i];
                            farthest2 = corners[i - 1];
                        }
                    }
                }

                if (corners.length != 4)
                    ang = lastAng;
                // Вычисляем угол наклона прямой через найденные точки
                else if (farthest1 != null && farthest2 != null && !allEqual && minRatio < 1 && minRatio > 0) {
                    tg = (farthest1.get(1) - farthest2.get(1)) / (farthest1.get(0) - farthest2.get(0));
                    ang = Math.toDegrees(Math.atan(tg));
                    Pose3D pose = colorResult.getTargetPoseCameraSpace();
                }
                else if (allEqual && minRatio < 1 && minRatio > 0)
                    ang = 90;
                else
                    ang = lastAng;

                for (double dist : dists)
                    telemetry.addLine(dist + "");
                telemetry.addData("x deg", xDeg);
                telemetry.addData("y deg", yDeg);
                telemetry.addData("area", area);
                telemetry.addData("dist", maxDistance);
                telemetry.addData("ang", ang);
                telemetry.addData("test", 1);
                telemetry.addData("allEq", allEqual);
                telemetry.addData("minRatio", minRatio);
                telemetry.addData("cornersAmt", corners.length);
                telemetry.update();
                diffy.setRoll(angToDif(ang));
                diffy.update();
                lastAng = ang;
            }
        }
    }

    private static double calculateDistance(List<Double> p1, List<Double> p2) {
        return Math.sqrt(Math.pow(p1.get(0) - p2.get(0), 2) + Math.pow(p1.get(1) - p2.get(1), 2));
    }

    private double angToDif(double ang)
    {
        if (ang < 0)
            return 80 + ang;
        if (ang < 80)
            return -80 + ang;
        if (ang == 90)
            return 0;
        return 0;
    }
}