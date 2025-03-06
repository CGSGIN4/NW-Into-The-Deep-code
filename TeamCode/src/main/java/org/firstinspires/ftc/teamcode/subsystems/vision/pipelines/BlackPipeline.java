package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.vision.Sample;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class BlackPipeline implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private Sample nearestSample = new Sample(-1, -1, new Point(0, 0), -1, Sample.SampleColor.UNDETECTED);
    public static Scalar lower = new Scalar(0, 0, 160);
    public static Scalar upper = new Scalar(0, 0, 250);
    public Mat findCorners(Mat image) {
        Mat gray = new Mat();
        Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);

        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, 50, 150);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//        Imgproc.drawContours(image, contours, -1, new Scalar(255, 0, 255));

        nearestSample = new Sample(-1, -1, new Point(0, 0), -1, Sample.SampleColor.UNDETECTED);
        Mat result = image.clone();
        double maxArea = 0;
        for (MatOfPoint cnt : contours) {
            double area = Imgproc.contourArea(cnt);
            if (area < 1000 || area > 100000) continue;

            RotatedRect box = Imgproc.minAreaRect(new MatOfPoint2f(cnt.toArray()));

            double width = box.size.width;
            double height = box.size.height;
            double angle = box.angle;

            if (width > height) {
                angle += 90;
            }

            if (area > maxArea) {
                maxArea = area;
                this.nearestSample = new Sample(width, height, new Point(320 - box.center.x, box.center.y - 240), angle, Sample.SampleColor.YELLOW);

                Point[] boxPoints = new Point[4];
                box.points(boxPoints);

                for (int j = 0; j < 4; ++j) {
                    Imgproc.line(result, boxPoints[j], boxPoints[(j + 1) % 4], new Scalar(255, 0, 0), 5);
                }
            }
        }
        gray.release();
        edges.release();
        hierarchy.release();
        return result;
    }

    public Sample getNearestSample() {
        return nearestSample;
    }

    static double getDistance(Point p1, Point p2) {
        return Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    static public int pixelToTicks(double px) {
        return (int) Math.round(0.69459474625 * px + 543.39459234142);
    }

    static public double pixelToInchesY(double px){
        return -0.01405882764 * px - 1.24327042537;
    }

    static public double pixelToInchesX(double px){
        return 0.01455155735 * px + 11.22243094813;
    }

    @Override
    public void init(int i, int i1, CameraCalibration cameraCalibration) {

    }

    @Override
    public Object processFrame(Mat image, long captureTimeNanos) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(image, hsv, Imgproc.COLOR_BGR2HSV);

        Mat mask = new Mat();

        //Scalar lower = new Scalar(160, 0, 142);
        //Scalar upper = new Scalar(200, 250, 250);

        Core.inRange(hsv, lower, upper, mask);

        Mat filtered = new Mat();
        Core.bitwise_and(image, image, filtered, mask);
        Mat cornersImage = findCorners(filtered);

        cornersImage.copyTo(image);

        cornersImage.release();
        hsv.release();
        mask.release();
        filtered.release();

        Imgproc.circle(image, new Point(-nearestSample.getCenter().x + 320, nearestSample.getCenter().y + 240), 25, new Scalar(255, 0, 0));

        Imgproc.putText(
                image,
                String.format("%s", nearestSample.getArea()),
                new Point(60, 90),
                Imgproc.FONT_HERSHEY_COMPLEX,
                1,
                new Scalar(255, 255, 255),
                2
        );

        Bitmap b = Bitmap.createBitmap(image.width(), image.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(image, b);
        lastFrame.set(b);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
