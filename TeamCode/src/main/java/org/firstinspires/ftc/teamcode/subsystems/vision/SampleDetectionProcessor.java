package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.vision.Sample.SampleColor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class SampleDetectionProcessor implements VisionProcessor {

    final static Scalar YELLOW = new Scalar(255, 255, 0);
    final static Scalar RED = new Scalar(255, 0, 0);
    final static Scalar BLUE = new Scalar(0, 0, 255);
    final static Scalar BLACK = new Scalar(0, 0, 0);
    final static Scalar WHITE = new Scalar(255, 255, 255);
    final static Scalar GREEN = new Scalar(0, 255, 0);

    final static double MIN_AREA = 1000;
    final static double MAX_AREA = 200000;

    static final int YELLOW_MASK_THRESHOLD = 80;
    static final int BLUE_MASK_THRESHOLD = 160;
    static final int RED_MASK_THRESHOLD = 190;

    // Here we just hope that largest area contour means nearest contour :)
    Sample tempNearestSample = new Sample(0, 0, new Point(0, 0), 0, SampleColor.UNDETECTED);
    Sample resultNearestSample = new Sample(0, 0, new Point(0, 0), 0, SampleColor.UNDETECTED);

    public Sample getNearestSample() {
        return resultNearestSample;
    }

    private void finder(Mat frame, List<MatOfPoint> contours, SampleColor color) {
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);

            RotatedRect box = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

            double width = box.size.width;
            double height = box.size.height;
            double angle = box.angle;

            if (width > height) {
                angle += 90;
            }

            if (width * height < MIN_AREA) continue;
            if (width * height > MAX_AREA) continue;

            Point[] boxPoints = new Point[4];
            box.points(boxPoints);

            MatOfPoint matOfPoint = new MatOfPoint();
            Imgproc.boxPoints(box, matOfPoint);

            for (int j = 0; j < 4; ++j) {
                Imgproc.line(frame, boxPoints[j], boxPoints[(j + 1) % 4], WHITE, 1);
            }

            if (width * height > tempNearestSample.getArea()) {
                tempNearestSample = new Sample(
                        width,
                        height,
                        Sample.toCentricCoordinates(frame.width(), frame.height(), box.center),
                        angle,
                        color
                );
            }
        }

        resultNearestSample = tempNearestSample;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        tempNearestSample = new Sample(0, 0, new Point(0, 0), 0, SampleColor.UNDETECTED);

        Mat ycrcbMat = new Mat();
        Mat cbMat = new Mat();
        Mat crMat = new Mat();
        Mat yMat = new Mat();

        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(ycrcbMat, cbMat, 2); // Cb channel index is 2
        Core.extractChannel(ycrcbMat, crMat, 1); // Cr channel index is 1

        Mat blueMask = new Mat();
        Mat redMask = new Mat();
        Mat yellowMask = new Mat();

        Imgproc.threshold(cbMat, blueMask, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(crMat, redMask, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(cbMat, yellowMask, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        List<MatOfPoint> blueContours = new ArrayList<>();
        List<MatOfPoint> yellowContours = new ArrayList<>();
        List<MatOfPoint> redContours = new ArrayList<>();

        Imgproc.findContours(blueMask, blueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_L1);
        Imgproc.findContours(redMask, redContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_L1);
        Imgproc.findContours(yellowMask, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        finder(frame, blueContours, SampleColor.BLUE);
        finder(frame, redContours, SampleColor.RED);
        finder(frame, yellowContours, SampleColor.YELLOW);

        if (tempNearestSample.getColor() != SampleColor.UNDETECTED) {
            Imgproc.drawMarker(frame, tempNearestSample.toOpenCVCoordinates(frame.width(), frame.height()), GREEN);
            Imgproc.putText(
                    frame,
                    String.format("(%s, %s) | %s deg",
                            Math.ceil(tempNearestSample.getCenter().x),
                            Math.ceil(tempNearestSample.getCenter().y),
                            Math.ceil(tempNearestSample.getAngle())
                    ),
                    new Point(60, 60),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    1,
                    new Scalar(0, 0, 0),
                    2
            );
            Imgproc.putText(
                    frame,
                    String.format("%s x %s | %s |",
                            Math.ceil(tempNearestSample.getWidth()),
                            Math.ceil(tempNearestSample.getHeight()),
                            tempNearestSample.getColor()
                    ),
                    new Point(60, 90),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    1,
                    new Scalar(0, 0, 0),
                    2
            );
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}