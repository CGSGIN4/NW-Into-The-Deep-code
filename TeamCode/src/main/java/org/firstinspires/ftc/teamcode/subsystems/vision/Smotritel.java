package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.BlackPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

public class Smotritel {
    BlackPipeline pipeline;
    VisionPortal portal;

    public Smotritel(HardwareMap hardwareMap) {
        this.pipeline = new BlackPipeline();

        WebcamName camName = hardwareMap.get(WebcamName.class, "cam");
        this.portal = new VisionPortal.Builder()
                .addProcessor(pipeline)
                .setCameraResolution(new Size(640, 480))
                .setCamera(camName)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .build();
    }

    // width and height is shit do not believe!!!
    public Sample getNearestSample() {
        return pipeline.getNearestSample();
    }

    public void startStream() {
        portal.resumeStreaming();
    }

    public void stopStream() {
        portal.stopStreaming();
    }

    public void startDashboardStream() {
        FtcDashboard.getInstance().startCameraStream(pipeline, 0);
    }

    public void stopDashboardStream() {
        FtcDashboard.getInstance().stopCameraStream();
    }

}
