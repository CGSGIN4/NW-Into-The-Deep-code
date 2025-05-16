package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.modules.differential;
import org.firstinspires.ftc.teamcode.subsystems.path_follower;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.util.Arrays;

@Config
@TeleOp
public class LimeLightPyTest1 extends LinearOpMode {
    private Limelight3A ll;
    public static double tolerance = 0.7;
    @Override
    public void runOpMode() throws InterruptedException {
        differential differential = new differential(hardwareMap);
        ll = hardwareMap.get(Limelight3A.class, "limelight");

        Robot robot = new Robot(hardwareMap);
        robot.init();
        dataStorage.init(robot.drive, telemetry, this);
        path_follower path_follower = new path_follower(robot.drivetrain);
        robot.drive.setPoseEstimate(new Pose2d(39.9, 64.93, Math.PI));

        telemetry.setMsTransmissionInterval(11);

        waitForStart();

        ll.start();

        double[] ang = {-1};
        while (opModeIsActive()) {
            ll.pipelineSwitch(1); // !!! python pipeline is 1 !!!
            LLResult result = ll.getLatestResult();

            double w = 0;
            double h = 0;


            double[] inputs = { robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
            ll.updatePythonInputs(inputs);
            ll.updatePythonInputs(inputs);
            ll.updatePythonInputs(inputs);

            if (result != null) {
                /* !!! llpython = [1, cx, cy, w, h, ang, 0, 0] !!! */
                double[] pyRes = result.getPythonOutput();

                if (pyRes != null && pyRes.length > 0) {
                    ang = pyRes;
                }
            }

//            telemetry.addData("ang", ang);
//            telemetry.addData("w", w);
//            telemetry.addData("h", h);


//            telemetry.addData("pipeline num", ll.getLatestResult().getPipelineIndex());
//            telemetry.addData("pipeline type", ll.getLatestResult().getPipelineType());
//            telemetry.addData("pipeline type", ll.getStatus().toString());
            telemetry.addData("res", Arrays.toString(ang));
            telemetry.addData("x", robot.drive.getPoseEstimate().getX());
            telemetry.addData("y", robot.drive.getPoseEstimate().getY());
            telemetry.update();
            robot.drive.update();
        }

    }
}
