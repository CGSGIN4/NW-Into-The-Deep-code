package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class ai_samples extends OpenCvPipeline{

    @Override
    public Mat processFrame(Mat image) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(image, hsv, Imgproc.COLOR_BGR2HSV);

        Mat mask = new Mat();
        Scalar lower = new Scalar(160, 0, 200);
        Scalar upper = new Scalar(200, 5, 250);

        //Scalar lower = new Scalar(160, 0, 142);
        //Scalar upper = new Scalar(200, 250, 250);

        Core.inRange(hsv, lower, upper, mask);

        Mat filtered = new Mat();
        Core.bitwise_and(image, image, filtered, mask);
        Mat cornersImage = findCorners(filtered);
        return cornersImage;
    }

    public static Mat findCorners(Mat image) {
        Mat gray = new Mat();
        Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);

        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, 50, 150);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(image, contours, -1, new Scalar(255, 0, 255));

        Mat result = image.clone();
        for (MatOfPoint cnt : contours) {
            double area = Imgproc.contourArea(cnt);
            if (area < 10 || area > 750) continue;

            Point rightmost = null, leftmost = null, topmost = null, bottommost = null;
            for (Point point : cnt.toArray()) {
                if (rightmost == null || point.x > rightmost.x) rightmost = point;
                if (leftmost == null || point.x < leftmost.x) leftmost = point;
                if (topmost == null || point.y < topmost.y) topmost = point;
                if (bottommost == null || point.y > bottommost.y) bottommost = point;
            }

            if (rightmost != null && leftmost != null && topmost != null && bottommost != null) {
                double minDistance = 0.0;
                if (getDistance(rightmost, leftmost) > minDistance &&
                        getDistance(rightmost, topmost) > minDistance &&
                        getDistance(rightmost, bottommost) > minDistance &&
                        getDistance(topmost, leftmost) > minDistance &&
                        getDistance(bottommost, leftmost) > minDistance &&
                        getDistance(topmost, bottommost) > minDistance) {

                    Imgproc.circle(result, rightmost, 2, new Scalar(0, 0, 255), -1);
                    Imgproc.circle(result, leftmost, 2, new Scalar(0, 255, 0), -1);
                    Imgproc.circle(result, topmost, 2, new Scalar(255, 0, 0), -1);
                    Imgproc.circle(result, bottommost, 2, new Scalar(255, 255, 0), -1);

                    Imgproc.line(result, rightmost, leftmost, new Scalar(0, 255, 255), 1);
                    Imgproc.line(result, bottommost, topmost, new Scalar(255, 0, 255), 1);
                }
            }
        }
        return result;
    }

    static double getDistance(Point p1, Point p2) {
        return Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }
}
