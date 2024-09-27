package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.utils.ring_buffer;

@TeleOp
public class ring_buf_test extends LinearOpMode {
    Robot robot;
    ring_buffer<Double> buf = new ring_buffer<>();
    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);
        robot.init();
        dataStorage.init(new SampleMecanumDrive(hardwareMap), telemetry, this);
        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a) {
                telemetry.addData("offer", buf.offer(Math.random() * 358));
                telemetry.update();
            }
            if (gamepad1.b) {
                telemetry.addData("read", buf.get());
                telemetry.update();
            }
            if (gamepad1.x) {
                telemetry.addData("capacity", buf.getCapacity());
                telemetry.update();
            }
            if (gamepad1.y) {
                double n = Math.random() * 800;
                buf.put(Math.random() * 800);
                telemetry.addData("added", n);
                telemetry.update();
            }
            if (gamepad1.right_bumper) {
                buf.output();
            }
        }
    }

}
