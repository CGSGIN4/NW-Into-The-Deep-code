package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.opencv.core.Point;

public class Sample {
    public enum SampleColor {
        RED,
        BLUE,
        YELLOW,
        UNDETECTED
    }

    private SampleColor color;

    // openCV thinks (0, 0) point is in left up corner
    // I believe it is in the center of the image
    // point below using my coordinate system!!!
    private final Point center;

    private final double width; // pixels
    private final double height; // pixels
    private final double angle; //degrees

    public static Point toCentricCoordinates(int frameWidth, int frameHeight, Point openCVPoint) {
        return new Point(openCVPoint.x - (double) frameWidth / 2, (double) frameHeight / 2 - openCVPoint.y);
    }

    /**
     * @param width  in pixels
     * @param height in pixels
     * @param center where (0,0) is the center of the screen
     * @param angle  in degrees
     * @param color  Red, Blue, Yellow or nothing (undetected)
     */
    public Sample(double width, double height, Point center, double angle, SampleColor color) {
        this.width = width;
        this.height = height;
        this.center = center;
        this.color = color;
        this.angle = angle;
    }

    public Point getCenter() {
        return center;
    }

    public SampleColor getColor() {
        return color;
    }

    public double getHeight() {
        return height;
    }

    public double getWidth() {
        return width;
    }

    public double getArea() {
        return width * height;
    }

    public double getAngle() {
        return angle;
    }

    /**
     * Convert sample coordinates from centric-based system to openCV system
     * where (0,0) is in the left up corner of the screen.
     */
    public Point toOpenCVCoordinates(int frameWidth, int frameHeight) {
        return new Point(center.x + (double) frameWidth / 2, -center.y + (double) frameHeight / 2);
    }
}