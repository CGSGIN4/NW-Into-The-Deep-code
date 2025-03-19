package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.modules.differential;

import java.util.Arrays;

@Config
@TeleOp
public class LimeLightPyTest extends LinearOpMode {
    private Limelight3A ll;
    public static double tolerance = 0.7;
    @Override
    public void runOpMode() throws InterruptedException {
        differential differential = new differential(hardwareMap);
        ll = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);


        ll.pipelineSwitch(1); // !!! python pipeline is 1 !!!
        ll.pipelineSwitch(1); // !!! python pipeline is 1 !!!
        ll.pipelineSwitch(1); // !!! python pipeline is 1 !!!
        waitForStart();

        differential.setRoll(-11);
        differential.setPitch(90);
        differential.update();

        ll.start();

        while (!(ll.isRunning() && ll.isConnected())) {}

        double ang = 0;
        while (opModeIsActive()) {
            ll.pipelineSwitch(1); // !!! python pipeline is 1 !!!
            LLResult result = ll.getLatestResult();

            double w = 0;
            double h = 0;


            if (result != null) {
                /* !!! llpython = [1, cx, cy, w, h, ang, 0, 0] !!! */
                double[] pyRes = result.getPythonOutput();

                if (pyRes != null && pyRes.length > 0) {
                    w = pyRes[3];
                    h = pyRes[4];
                    ang = pyRes[5];
                }
            }

            differential.setRoll(-80 + ang * (8.0 / 9));
            differential.update();

            telemetry.addData("ang", ang);
            telemetry.addData("w", w);
            telemetry.addData("h", h);


//            telemetry.addData("pipeline num", ll.getLatestResult().getPipelineIndex());
//            telemetry.addData("pipeline type", ll.getLatestResult().getPipelineType());
//            telemetry.addData("pipeline type", ll.getStatus().toString());
            telemetry.update();
        }

    }
}
