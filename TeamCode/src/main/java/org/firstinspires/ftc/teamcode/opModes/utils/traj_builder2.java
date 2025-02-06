package org.firstinspires.ftc.teamcode.opModes.utils;

import android.os.Environment;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.utils.painter;
import org.firstinspires.ftc.teamcode.utils.parser;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(group = "Utils")
public class traj_builder2 extends LinearOpMode {
    String FILENAME = "b19dotaPRO";
    private FtcDashboard dashboard;
    int MAX_CURVES = 30;
    parser parser = new parser(FILENAME);
    double curve_index = 0;
    double t = 0;
    int numCurves = 0;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        dataStorage.init(drive, telemetry, this);
        painter painter = new painter();
        int activePointControl = 0;
        int activeCurveControl = 0;
        curve[] curves = new curve[MAX_CURVES];

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();
        drive.update();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            TelemetryPacket packet = new TelemetryPacket(true);
            Canvas fieldOverlay = packet.fieldOverlay();

            painter.prepare(packet, fieldOverlay);

            if (currentGamepad1.a && !previousGamepad1.a){
                activePointControl += 1;
                if (activePointControl == 4){
                    if (activeCurveControl < numCurves - 1) {
                        activePointControl = 1;
                        activeCurveControl++;
                    }
                    else {
                        activePointControl = 0;
                        activeCurveControl = 0;
                    }
                }
            }

            else if (currentGamepad1.b && !previousGamepad1.b) {
                activePointControl -= 1;
                if (activePointControl == -1){
                    if (activeCurveControl == 0) {
                        activePointControl = 3;
                        activeCurveControl = numCurves - 1;
                    }
                    else {
                        activePointControl = 3;
                        activeCurveControl -= 1;
                    }
                }
            }
            else if (currentGamepad1.x && !previousGamepad1.x) {
                curves[numCurves - 1] = null;
                if (activeCurveControl == numCurves - 1)
                    activeCurveControl--;
                numCurves--;
            }
            else if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                t += 0.02;
                if (t >= 1.) {
                    if (activeCurveControl < numCurves - 1)
                        activeCurveControl++;
                    else
                        activeCurveControl = 0;
                    t = 0;
                }
                t = (double)Math.round(t * 100) / 100;
            }
            else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                t -= 0.02;
                if (t <= 0.) {
                    if (activeCurveControl > 0)
                        activeCurveControl--;
                    else
                        activeCurveControl = 0;
                    t = 1;
                }
                t = (double)Math.round(t * 100) / 100;
            }
            else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                if (!curves[activeCurveControl].turn_starts.contains(t))
                    curves[activeCurveControl].turn_starts.add(t);
                else
                    curves[activeCurveControl].turn_starts.remove(t);
            }
            else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                if (!curves[activeCurveControl].turn_ends.contains(t))
                    curves[activeCurveControl].turn_ends.add(t);
                else
                    curves[activeCurveControl].turn_ends.remove(t);
            }
            else if (currentGamepad1.y && !previousGamepad1.y) {
                if (numCurves != 0) {
                    try {
                        curves[numCurves] = new curve(new Vector2d[]{curves[numCurves - 1].nodes[3], new Vector2d(0, 10), new Vector2d(0, 20), new Vector2d(0, 30)}, 50);
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                } else {
                    try {
                        curves[numCurves] = new curve(new Vector2d[]{new Vector2d(0, 0), new Vector2d(0, 10), new Vector2d(0, 20), new Vector2d(0, 30)}, 50);
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
                numCurves++;
                activePointControl = 0;
                activeCurveControl = numCurves - 1;
            }
            else if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1){
                Vector2d gamepad = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).times(0.06);
                if (curves[activeCurveControl] != null) {
                    curves[activeCurveControl].nodes[activePointControl] = curves[activeCurveControl].nodes[activePointControl].plus(gamepad);
                }

                if (activePointControl == 3 && activeCurveControl != numCurves - 1){
                    curves[activeCurveControl + 1].nodes[0] = curves[activeCurveControl + 1].nodes[0].plus(gamepad);
                    curve tmp;
                    try {
                        tmp = new curve(curves[activeCurveControl + 1].nodes, 50);
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                    tmp.turn_ends = curves[activeCurveControl + 1].turn_ends;
                    tmp.turn_starts = curves[activeCurveControl + 1].turn_starts;
                    curves[activeCurveControl + 1] = tmp;
                }
                else if (activePointControl == 0 && activeCurveControl != 0){
                    curves[activeCurveControl - 1].nodes[3] = curves[activeCurveControl].nodes[0].plus(gamepad);
                    curve tmp;
                    try {
                        tmp = new curve(curves[activeCurveControl - 1].nodes, 50);
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                    tmp.turn_ends = curves[activeCurveControl - 1].turn_ends;
                    tmp.turn_starts = curves[activeCurveControl - 1].turn_starts;
                    curves[activeCurveControl - 1] = tmp;
                }

                curve tmp;
                try {
                    tmp = new curve(curves[activeCurveControl].nodes, 50);
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
                tmp.turn_ends = curves[activeCurveControl].turn_ends;
                tmp.turn_starts = curves[activeCurveControl].turn_starts;
                curves[activeCurveControl] = tmp;
            }

            /* draw grounds */
            painter.drawGround(-24, 24, "black");
            painter.drawGround(0, 48, "red");
            painter.drawGround(24, 24, "green");

            /* draw curves and their points*/
            for (int i = 0; i < numCurves; i++){
                if (i != activeCurveControl)
                    painter.drawPolyLine(curves[i].points, "white");
                else
                    painter.drawPolyLine(curves[i].points, "red");

                for (int j = 0; j < 4; j++) {
                    if (activeCurveControl != i || activePointControl != j)
                        painter.drawPoint(curves[i].nodes[j].getX(), curves[i].nodes[j].getY(), "white");
                    else
                        painter.drawPoint(curves[i].nodes[j].getX(), curves[i].nodes[j].getY(), "orange");
                }
                painter.drawPoint(curves[activeCurveControl].getPoint(t).getX(), curves[activeCurveControl].getPoint(t).getY(), "purple");
            }

            /* draw start and end turn points */
            if (curves[activeCurveControl] != null) {
                for (double t : curves[activeCurveControl].turn_starts) {
                    painter.drawPoint(curves[activeCurveControl].getPoint(t).getX(), curves[activeCurveControl].getPoint(t).getY(), "green");
                }
                for (double t : curves[activeCurveControl].turn_ends) {
                    painter.drawPoint(curves[activeCurveControl].getPoint(t).getX(), curves[activeCurveControl].getPoint(t).getY(), "red");
                }
            }

            int index = 0;
            for (curve curve : curves)
            {
                if (curve != null){
                    for (Vector2d node : curve.nodes){
                        packet.put("node " + index, node.toString());
                        index++;
                    }
                }
            }

            dashboard.sendTelemetryPacket(packet);

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                try {
                    File directory = new File(String.format("%s/FIRST/points", Environment.getExternalStorageDirectory().getPath()));
                    directory.mkdir();

                    FileWriter fw = new FileWriter(String.format("%s/FIRST/points/%s.csv", Environment.getExternalStorageDirectory().getPath(), FILENAME), false);
                    StringBuilder str = buildFileContents(curves);

                    fw.write(str.toString());
                    telemetry.addData("str", str.substring(0));
                    telemetry.update();
                    fw.flush();
                    fw.close();
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            }
            if (gamepad1.left_bumper){
                try {
                    index = 0;
                    for (curve curve : parser.getCurves()){
                        if (curve != null) {
                            curves[index] = curve;
                            index++;
                        }
                    }

                    activeCurveControl = 0;
                    numCurves = index;
                    dataStorage.DSTelemetry.addData("num curves", numCurves);
                    dataStorage.DSTelemetry.update();
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    @NonNull
    private static StringBuilder buildFileContents(curve[] curves) {
        StringBuilder str = new StringBuilder();

        for (curve curve : curves){
            if (curve != null) {
                for (Vector2d node : curve.nodes) {
                    str.append(node.toString()).append(";");
                }

                //str.deleteCharAt(str.lastIndexOf(";"));

                for (double t : curve.turn_starts) {
                    str.append(t).append(";");
                }

                for (double t : curve.turn_ends) {
                    str.append(t).append(";");
                }

                str.append("\n");
            }
        }
        return str;
    }
}
