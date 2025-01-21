package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class vertice_method extends OpenCvPipeline {

    // Threshold values for HSV filtering for red, blue, yellow
    private Scalar lowerBound;
    private Scalar upperBound;
    private Scalar lowerBound1;
    private Scalar upperBound1;
    private TargetColor targetColor = TargetColor.RED;

    // Target color identifier
    public enum TargetColor {
        RED, BLUE, YELLOW
    }

    public vertice_method() {
        // Set HSV threshold ranges based on the target color
        switch (targetColor) {
            case RED:
                // Red color thresholds in HSV
                lowerBound = new Scalar(0, 100, 90);  // Lower bound of red
                upperBound = new Scalar(3, 255, 255); // Upper bound of red
                lowerBound1 = new Scalar(170, 240, 20);  // Lower bound of red
                upperBound1 = new Scalar(180, 255, 255); // Upper bound of red
                break;
            case BLUE:
                // Blue color thresholds in HSV
                lowerBound = new Scalar(100, 90, 5);  // Lower bound of blue
                upperBound = new Scalar(140, 255, 255); // Upper bound of blue
                break;
            case YELLOW:
                // Yellow color thresholds in HSV
                lowerBound = new Scalar(8, 240, 230);  // Lower bound of yellow
                upperBound = new Scalar(40, 255, 255); // Upper bound of yellow
                break;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert image to HSV for better color segmentation
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Threshold the HSV image to isolate the target color
        Mat mask = new Mat();
        Core.inRange(hsv, lowerBound, upperBound, mask);
        Mat mask2 = new Mat();
        if (targetColor == TargetColor.RED)
        {
            Core.inRange(hsv, lowerBound1, upperBound1, mask2);
            Core.bitwise_or(mask, mask2, mask);
        }

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 255));

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(contour2f);

            // Get dimensions of the rotated rectangle
            Size rectSize = rect.size;

            // Check if the contour matches the expected size (1.5" x 3")
            double aspectRatio = Math.max(rectSize.width, rectSize.height) / Math.min(rectSize.width, rectSize.height);
            if ((rectSize.width > 20 || rectSize.height > 10) && rectSize.width < 100 && rectSize.height < 50) {
                // Store detected element
                //detectedElements.add(rect);

                // Draw the rectangle on the input frame
                Point[] vertices = new Point[4];
                rect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                // Draw orientation line (longer side)
                Point center = rect.center;
                double angle = rect.angle;

                // Calculate endpoint for orientation line
                double length = Math.max(rectSize.width, rectSize.height) / 2;
                double angleRad = Math.toRadians(angle);
                Point endpoint = new Point(
                        center.x + length * Math.cos(angleRad),
                        center.y + length * Math.sin(angleRad)
                );

                // Draw orientation line
                Imgproc.line(input, center, endpoint, new Scalar(255, 0, 0), 2);
            }
        }

        return input;
    }
}
