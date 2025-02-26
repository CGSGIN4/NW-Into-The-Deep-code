package org.firstinspires.ftc.teamcode.opModes.tests;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm.rotation;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm.extension;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.BlackPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;

@TeleOp
@Config
public class VisionTest extends LinearOpMode {

    public static double speed = -0.12;
    public static int WBC = 0;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.init();

        module_master.init(hardwareMap);

        DcMotor extMotor = hardwareMap.get(DcMotor.class, "armExtensionMotor");
        extMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dataStorage.init(bot.drive, telemetry, this);
        bot.drive.setPoseEstimate(new Pose2d(39.9, 64.93, Math.PI));
        path_follower follower = new path_follower(bot.drivetrain);


        int whenSeen = 0;
        int timesSeen = 0;
//
//        diff.setRoll(-8);
//        diff.setPitch(112);
//        diff.update();



        BlackPipeline sampleDetection = new BlackPipeline();

        WebcamName camName = hardwareMap.get(WebcamName.class, "cam");

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(sampleDetection)
                .setCameraResolution(new Size(640, 480))
                .setCamera(camName)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        waitForStart();

        module_master.arm.setRotation(rotation.CAMERA);
        module_master.arm.setExtension(extension.CAMERA);

        module_master.differential.openClaw();
        module_master.differential.setRoll(-10);
        module_master.differential.setPitch(73);
        module_master.differential.update();

        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("waiting for camera to start stream", portal.getCameraState().toString());
        }

//        extMotor.setPower(speed);

        double rememberAng = 0;
        int rememberDist = 0;
        module_master.arm.resetRotationEncoders();
        module_master.arm.setRotation(rotation.FRONT);
        while (opModeIsActive()) {
            telemetry.addData("nearestAng", sampleDetection.getNearestAng());
            telemetry.addData("nearest x", sampleDetection.getNearestCenter().x);
            telemetry.addData("nearest y", sampleDetection.getNearestCenter().y);
            telemetry.update();

            if (gamepad1.cross) {
                dataStorage.updateData();
                double yOffset = 0, xOffset = 0;
                rememberAng = sampleDetection.getNearestAng();
                yOffset = BlackPipeline.pixelToInchesY(sampleDetection.getNearestCenter().y);
                rememberDist = BlackPipeline.pixelToTicks(sampleDetection.getNearestCenter().x);
                module_master.arm.ROTATION_PIDF = new PIDFCoefficients(0.0026, 0, 0.0019, -0.0032);
                module_master.arm.pidExtend(rememberDist);
                module_master.arm.setRotation(rotation.FRONT);
                follower.goToPos(dataStorage.RobotWorldX, dataStorage.RobotWorldY + yOffset, dataStorage.RobotWorldHeading);

                while (!module_master.arm.extensionReached() || !module_master.arm.rotationReached())
                    module_master.update(dataStorage.telemetry);
                module_master.differential.pitchDown();
                module_master.differential.setRoll(differential.geomToDifAngle(rememberAng));
                module_master.differential.update();
                sleep(300);

                module_master.differential.closeClaw();
                sleep(200);
                module_master.differential.pitchUp();
                module_master.arm.setExtension(extension.CLOSED);
            }

            if (gamepad1.square) {
                module_master.arm.ROTATION_PIDF = new PIDFCoefficients(0.0026, 0, 0.0019, -0.0182);
                module_master.arm.setRotation(rotation.CAMERA);
                module_master.arm.setExtension(extension.CAMERA);
                module_master.differential.openClaw();
                module_master.differential.setRoll(-10);
                module_master.differential.setPitch(73);
                module_master.differential.update();
            }

            if (gamepad1.triangle) {
                module_master.arm.pidExtend(rememberDist);
            }

            if(gamepad1.circle) {
                module_master.differential.pitchDown();
                sleep(300);
                module_master.differential.setRoll(differential.geomToDifAngle(rememberAng));
                module_master.differential.update();
                sleep(400);
                module_master.differential.closeClaw();
                sleep(200);
                module_master.differential.pitchUp();
                module_master.arm.setExtension(extension.CLOSED);
            }

            module_master.update(dataStorage.telemetry);
        }
    }
}
