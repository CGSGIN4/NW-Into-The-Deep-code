package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.vision.Sample.SampleColor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
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
import java.util.concurrent.atomic.AtomicReference;

public class SampleDetectionProcessor implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    final static Scalar YELLOW = new Scalar(255, 255, 0);
    final static Scalar RED = new Scalar(255, 0, 0);
    final static Scalar BLUE = new Scalar(0, 0, 255);
    final static Scalar BLACK = new Scalar(0, 0, 0);
    final static Scalar WHITE = new Scalar(255, 255, 255);
    final static Scalar GREEN = new Scalar(0, 255, 0);

    final static double MIN_AREA = 30000;
    final static double MAX_AREA = 127300;
    final static double MIN_X = 20;
    final static double MIN_Y = 20;

    public static Scalar lowerBlue = new Scalar(55, 60, 20);
    public static Scalar upperBlue = new Scalar(144, 255, 255);

    public static Scalar lowerYellow = new Scalar(18, 0, 170);
    public static Scalar upperYellow = new Scalar(45, 255, 255);

    public static Scalar lowerRed = new Scalar(0, 0, 200);
    public static Scalar upperRed = new Scalar(16, 255, 255);

    // Here we just hope that largest area contour means nearest contour :)
    Sample tempNearestSample = new Sample(0, 0, new Point(0, 0), 0, SampleColor.UNDETECTED);
    Sample resultNearestSample = new Sample(0, 0, new Point(0, 0), 0, SampleColor.UNDETECTED);

    double imageTemprature = 0;

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

            Point center = Sample.toCentricCoordinates(frame.width(), frame.height(), box.center);

            if (width * height < MIN_AREA) continue;
            if (width * height > MAX_AREA) continue;
            // -230 - 100 x
            // > -57
            if (!(center.x > -200 && center.x < 70)) continue;
            if (!(center.y > -57)) continue;

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

    private double rgbFrameToTemp(Mat rgbFrame) {
        Mat labFrame = new Mat();
        Imgproc.cvtColor(rgbFrame, labFrame, Imgproc.COLOR_RGB2Lab);

        List<Mat> labChannels = new ArrayList<>();
        Core.split(labFrame, labChannels);

        Mat aChannel = labChannels.get(1);
        Mat bChannel = labChannels.get(2);

        Scalar meanA = Core.mean(aChannel);
        Scalar meanB = Core.mean(bChannel);

        return 5000 + (meanB.val[0] - meanA.val[0]) * 50;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Point center = new Point(frame.cols() / 2.0, frame.rows() / 2.0);
        Mat rotMat = Imgproc.getRotationMatrix2D(center, 50, 1.0);
        Imgproc.warpAffine(frame, frame, rotMat, frame.size());

        tempNearestSample = new Sample(0, 0, new Point(0, 0), 0, SampleColor.UNDETECTED);

//        frame.convertTo(frame, -1, 1, 20);
        imageTemprature = rgbFrameToTemp(frame);
//
//        if (imageTemprature > 5000) {
//            Mat blueFilter = new Mat(frame.size(), frame.type(), new Scalar(0, 0, 255));
//
//            double alpha = 0.25;
//            double beta = 1.0 - alpha;
//            Core.addWeighted(blueFilter, alpha, frame, beta, 0.0, frame);
//        }

        Mat hsvFrame = new Mat();

        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Mat blueMask = new Mat();
        Mat redMask = new Mat();
        Mat yellowMask = new Mat();

        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        Core.inRange(hsvFrame, lowerRed, upperRed, redMask);

//        redMask.copyTo(frame);

        List<MatOfPoint> blueContours = new ArrayList<>();
        List<MatOfPoint> yellowContours = new ArrayList<>();
        List<MatOfPoint> redContours = new ArrayList<>();

        Imgproc.findContours(blueMask, blueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(redMask, redContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(yellowMask, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        finder(frame, blueContours, SampleColor.BLUE);
        finder(frame, redContours, SampleColor.RED);
        finder(frame, yellowContours, SampleColor.YELLOW);

        if (/*tempNearestSample.getColor() != SampleColor.UNDETECTED*/ true) {
            Imgproc.drawMarker(frame, tempNearestSample.toOpenCVCoordinates(frame.width(), frame.height()), GREEN);
            Imgproc.putText(
                    frame,
                    String.format("(%s, %s) | %s deg | %s kel",
                            Math.ceil(tempNearestSample.getCenter().x),
                            Math.ceil(tempNearestSample.getCenter().y),
                            Math.ceil(180 - tempNearestSample.getAngle()),
                            Math.ceil(imageTemprature)
                    ),
                    new Point(60, 60),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    1,
                    WHITE,
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
                    WHITE,
                    2
            );
        }

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}