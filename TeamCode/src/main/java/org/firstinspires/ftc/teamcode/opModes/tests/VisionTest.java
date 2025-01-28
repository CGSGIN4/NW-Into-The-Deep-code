package org.firstinspires.ftc.teamcode.opModes.tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.vision.Sample;
import org.firstinspires.ftc.teamcode.subsystems.vision.SampleDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Config
public class VisionTest extends LinearOpMode {

    public static double speed = -0.09;
    public static int WBC = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        differential diff = new differential(hardwareMap);
        DcMotor extMotor = hardwareMap.get(DcMotor.class, "armExtensionMotor");
        extMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int whenSeen = 0;
        int timesSeen = 0;
//
        diff.openClaw();
        diff.rollDefault();
        diff.pitchForward();
        diff.update();
//        diff.setRoll(-8);
//        diff.setPitch(112);
//        diff.update();

        SampleDetectionProcessor sampleDetection = new SampleDetectionProcessor();

        WebcamName camName = hardwareMap.get(WebcamName.class, "cam");

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(sampleDetection)
                .setCameraResolution(new Size(640, 480))
                .setCamera(camName)
                .build();

        waitForStart();


        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("waiting for camera to start stream", portal.getCameraState().toString());
        }

        FtcDashboard.getInstance().startCameraStream(sampleDetection, 0);

        extMotor.setPower(speed);

        while (opModeIsActive()) {
            Sample nearestSample = sampleDetection.getNearestSample();
            double x = nearestSample.getCenter().x;
            double ang = nearestSample.getAngle();
            boolean nice = false;

            int whiteBalance = -1;

            telemetry.addData("wbc true", portal.getCameraControl(WhiteBalanceControl.class).setMode(WhiteBalanceControl.Mode.MANUAL));

            if (((ang > 0 && ang < 30) || (ang > 150 && ang < 180)) && x > 150) {
                nice = true;
            }

            if (ang > 30 && ang < 150 && x > 120) {
                nice = true;
            }

            telemetry.addData("angle", nearestSample.getAngle());
            telemetry.addData("x", nearestSample.getCenter().x);
            telemetry.addData("y", nearestSample.getCenter().y);
            telemetry.addData("color", nearestSample.getColor().toString());
            telemetry.addData("nice", nice);
            telemetry.addData("wbc", portal.getCameraControl(WhiteBalanceControl.class).getWhiteBalanceTemperature());
            telemetry.addData("state", portal.getCameraState().toString());

//            portal.getCameraControl(WhiteBalanceControl.class).setWhiteBalanceTemperature(100);

            if (extMotor.getCurrentPosition() < -730 ) {
                extMotor.setPower(0);
//                break;
            }

            if (nearestSample.getColor() == Sample.SampleColor.RED || nearestSample.getColor() == Sample.SampleColor.YELLOW)
            {
//                if (timesSeen == 0) {
//                    timesSeen = 1;
//                    extMotor.setPower(0.08);
//                    sleep(700);
//                } else {
                    extMotor.setPower(0);
                    ang += 90;

                    if (ang > 180) ang -= 180;

                    diff.pitchDown();
                    sleep(300);
                    diff.setRoll(differential.geomToDifAngle(ang));
                    diff.update();
                    sleep(300);
                    diff.closeClaw();
                    sleep(200);
                    diff.setRoll(-2);
                    diff.setPitch(100);
                    diff.update();
                    sleep(500);

                    break;
//                }
            }


            telemetry.addData("whenSeen", whenSeen);

            /*
            if (gamepad1.x && nearestSample.getColor() != Sample.SampleColor.UNDETECTED) {
                ang += 50;

                if (ang > 180) ang -= 180;

                diff.openClaw();
                diff.update();
                sleep(300);
                diff.pitchDown();
                diff.update();
                sleep(500);
                diff.setRoll(differential.geomToDifAngle(ang));
                diff.update();
                sleep(400);
                diff.closeClaw();
                diff.update();
                sleep(500);
                diff.setRoll(-2);
                diff.setPitch(100);
                diff.update();
            }*/







            if (gamepad1.a) {
                diff.openClaw();
                diff.update();
            }

            telemetry.update();
            sleep(20);
        }
    }
}
