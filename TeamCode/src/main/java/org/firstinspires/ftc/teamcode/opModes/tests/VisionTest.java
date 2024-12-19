package org.firstinspires.ftc.teamcode.opModes.tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.vision.SampleDetectionProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.Sample;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        differential diff = new differential(hardwareMap);

        diff.closeClaw();
        diff.pitchForward();
        diff.setRoll(differential.geomToDifAngle(90));
        diff.update();

        WebcamName cam = hardwareMap.get(WebcamName.class, "cam");

        SampleDetectionProcessor sampleDetection = new SampleDetectionProcessor();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(sampleDetection)
                .setCameraResolution(new Size(640, 480))
                .setCamera(cam)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            Sample nearestSample = sampleDetection.getNearestSample();
            boolean nice = Math.abs(nearestSample.getCenter().x) < 47 && nearestSample.getColor() != Sample.SampleColor.UNDETECTED;

            telemetry.addData("angle", nearestSample.getAngle());
            telemetry.addData("x", nearestSample.getCenter().x);
            telemetry.addData("y", nearestSample.getCenter().y);
            telemetry.addData("color", nearestSample.getColor().toString());
            telemetry.addData("nice", nice);

            if (gamepad1.x && nearestSample.getColor() != Sample.SampleColor.UNDETECTED) {
                diff.openClaw();
                diff.update();
                sleep(300);
                diff.pitchDown();
                diff.update();
                diff.setRoll(differential.geomToDifAngle(nearestSample.getAngle()));
                diff.update();
                sleep(400);
                diff.closeClaw();
                diff.update();
            }



            if (gamepad1.a) {
                diff.openClaw();
                diff.update();
                sleep(300);
                diff.pitchForward();
                diff.rollDefault();
                diff.update();

            }

            telemetry.update();
            sleep(20);
        }
    }
}
