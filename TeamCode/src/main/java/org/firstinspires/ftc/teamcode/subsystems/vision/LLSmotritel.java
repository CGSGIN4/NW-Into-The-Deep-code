package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.sql.Timestamp;
import java.util.List;

public class LLSmotritel {
    private Limelight3A ll;

    public LLSmotritel(HardwareMap hardwareMap, int defaultPPL) {
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(defaultPPL);
        ll.pipelineSwitch(defaultPPL);
        ll.pipelineSwitch(defaultPPL);
    }

    public LLSmotritel(HardwareMap hardwareMap) {
        ll = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void startStreaming() {
        ll.start();
    }

    public void startSnapshot() {
        Timestamp timestamp = new Timestamp(System.currentTimeMillis());
        ll.captureSnapshot("snap " + timestamp.toString());
    }

    public void deleteSnapshots() {
        ll.deleteSnapshots();
    }

    public void stopStreaming() {
        ll.pause();
    }

    public Pose2d getSampleOffsets(int pipeline) {
        /* Fuck this shit */
        ll.pipelineSwitch(pipeline);
        ll.pipelineSwitch(pipeline);
        ll.pipelineSwitch(pipeline);
        /* Stop fucking this shit */

        LLResult result = ll.getLatestResult();
        double w = 0;
        double h = 0;
        double ang = 90;

        if (result != null) {
            /* !!! llpython = [1, cx, cy, w, h, ang, 0, 0] !!! */
            double[] pyRes = result.getPythonOutput();

            if (pyRes != null && pyRes[0] != 0) {
                w = pyRes[3];
                h = pyRes[4];
                ang = pyRes[5];
                return new Pose2d(result.getTx(), 68.29 - result.getTy(), angToDif(ang));
            }
            return new Pose2d(-100, -100, 90);
        }
        return new Pose2d(-100, -100, 90);
    }

    private static double calculateDistance(List<Double> p1, List<Double> p2) {
        return Math.sqrt(Math.pow(p1.get(0) - p2.get(0), 2) + Math.pow(p1.get(1) - p2.get(1), 2));
    }

    public static int angToTicks(double ang) {
        //if (ang > 5 || ang < 0) return -1;
        return (int)Math.round(-0.80237561733 * ang * ang * ang + 1.13778911005 * ang * ang - 54.91040083883 * ang + 481.28050750253);
    }

    public static double ticksToInch(int ticks) {
        return 0.02121384408 * ticks + 0.16305403549;
    }

    public static int inchToTicks(double inch) {
        return (int)((inch - 0.16305403549) / 0.02121384408);
    }

    private double angToDif(double ang)
    {
        return -80 + ang * (8.0 / 9);
    }

    public double getTranslationalOffset(Pose2d offsets)
    {
        return - 7.8 * Math.tan(Math.toRadians(offsets.getY())) * Math.tan(Math.toRadians(offsets.getX()));
    }

    public static int getTicks(Pose2d offsets)
    {
        return inchToTicks(7.8 * Math.tan(Math.toRadians(offsets.getY())) - 13.5);
    }
}
