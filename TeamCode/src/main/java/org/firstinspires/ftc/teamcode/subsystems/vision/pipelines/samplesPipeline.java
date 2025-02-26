package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class samplesPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        // Preprocess the frame to detect yellow regions
        Mat yellowMask = preprocessFrame(input);

        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest yellow contour (blob)
        int largestContour = findLargestContour(contours);

        if (largestContour != -1) {
            Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 255));
            //for (MatOfPoint contour : contours)
                //Imgproc.rectangle(input, bbox(contour), new Scalar(0, 255, 255));
        }

        return input;
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Scalar lowerYellow = new Scalar(0, 0, 170);
        Scalar upperYellow = new Scalar(255, 255, 255);


        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        return yellowMask;
    }

    private int findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        int id = -1;
        for (MatOfPoint contour : contours) {
            id = contours.indexOf(contour);
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return id;
    }
    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private Rect bbox(MatOfPoint contour) {
        return Imgproc.boundingRect(contour);
    }

}
