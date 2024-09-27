package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class edge_detection extends OpenCvPipeline {

    private Scalar lowerHSV;
    private Scalar upperHSV;
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private List<RotatedRect> detectedElements = new ArrayList<>();

    public edge_detection() {
        String targetColor = "black";
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
    public Mat processFrame(Mat src) {
        // Convert image from RGB to HSV
        Mat gray = new Mat();
        Imgproc.cvtColor(src, gray, Imgproc.COLOR_BGR2GRAY);

        // Step 2: Apply Gaussian Blur to reduce noise
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(gray, blurred, new Size(5, 5), 0);

        // Step 3: Apply Canny Edge Detection
        Mat edges = new Mat();
        Imgproc.Canny(blurred, edges, 50, 150);

        // Step 4: Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(src, contours, -1, new Scalar(0, 255, 255));

        // Step 5: Filter contours that are rectangular
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            // Approximate the contour with a polygon
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, 0.02 * Imgproc.arcLength(contour2f, false), true);

            // If the approximated polygon has 4 vertices, it's a potential rectangle
            if (approx.total() == 4) {
                // Convert back to MatOfPoint for drawing
                MatOfPoint points = new MatOfPoint(approx.toArray());

                // Get the bounding box of the rectangle
                Rect rect = Imgproc.boundingRect(points);

                // Step 6: Draw rectangles on the source image
                Imgproc.rectangle(src, rect, new Scalar(0, 255, 0), 2);
            }
        }

        // Return the frame with drawings
        return src;
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

