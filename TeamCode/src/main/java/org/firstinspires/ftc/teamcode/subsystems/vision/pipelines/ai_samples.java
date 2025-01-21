package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class ai_samples extends OpenCvPipeline {

    private Scalar lowerHSV;
    private Scalar upperHSV;
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private List<RotatedRect> detectedElements = new ArrayList<>();

    public ai_samples() {
        String targetColor = "yellow";
        // Define color ranges based on the target color
        switch (targetColor.toLowerCase()) {
            case "red":
                lowerHSV = new Scalar(0, 90, 80);  // Lower bound of red
                upperHSV = new Scalar(3, 255, 255); // Upper bound of red
                break;
            case "blue":
                lowerHSV = new Scalar(100, 90, 5);  // Lower bound of blue
                upperHSV = new Scalar(140, 255, 255); // Upper bound of blue
                break;
            case "yellow":
                lowerHSV = new Scalar(8, 240, 230);  // Lower bound of yellow
                upperHSV = new Scalar(40, 255, 255); // Upper bound of yellow
                break;
            case "black":
                lowerHSV = new Scalar(0, 0, 0);  // Lower bound of yellow
                upperHSV = new Scalar(179, 180, 130); // Upper bound of yellow
                break;
            default:
                throw new IllegalArgumentException("Invalid color: " + targetColor);
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert image from RGB to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Apply color threshold
        Core.inRange(hsv, lowerHSV, upperHSV, mask);

        // Find contours of the detected elements
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 255));
        // Clear previous detections
        detectedElements.clear();

        // Iterate through contours and filter by shape and size
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(contour2f);

            // Get dimensions of the rotated rectangle
            Size rectSize = rect.size;

            // Check if the contour matches the expected size (1.5" x 3")
            double aspectRatio = Math.max(rectSize.width, rectSize.height) / Math.min(rectSize.width, rectSize.height);
            if ((rectSize.width > 30 || rectSize.height > 30) && rectSize.width < 100 && rectSize.height < 50) {
                // Store detected element
                detectedElements.add(rect);

                // Draw the rectangle on the input frame
                Point[] vertices = new Point[4];
                rect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                // Draw orientation line (longer side)
                Point center = rect.center;
                double angle = rect.angle;
                Imgproc.putText(input, angle + "", new Point(center.x, center.y - rectSize.height / 2), 16, 0.3, new Scalar(0, 255, 255));

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

        // Return the frame with drawings
        return input;
    }

    public List<RotatedRect> getDetectedElements() {
        return detectedElements;
    }

    public List<ElementData> getElementsData() {
        List<ElementData> elementsData = new ArrayList<>();
        for (RotatedRect rect : detectedElements) {
            ElementData data = new ElementData(rect.center.x, rect.center.y, rect.angle);
            elementsData.add(data);
        }
        return elementsData;
    }

    public static class ElementData {
        public double x;
        public double y;
        public double orientation;

        public ElementData(double x, double y, double orientation) {
            this.x = x;
            this.y = y;
            this.orientation = orientation;
        }

        @Override
        public String toString() {
            return "ElementData{" +
                    "x=" + x +
                    ", y=" + y +
                    ", orientation=" + orientation +
                    '}';
        }
    }
}

