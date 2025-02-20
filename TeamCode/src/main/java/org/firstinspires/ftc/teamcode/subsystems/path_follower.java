package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.robotMovement.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.modules.arm;
import org.firstinspires.ftc.teamcode.subsystems.modules.module_master;
import org.firstinspires.ftc.teamcode.utils.painter;

import java.util.HashMap;

public class path_follower {

    drivetrain drivetrain;
    public velocity_calculator velocity_calculator;
    public FtcDashboard dashboard;
    public painter painter = new painter();
    double p_rotation_coef = 3.;
    double p_trans_coef = 0.15;
    double d_trans_coef = 0.12;
    double i_trans_coef = 0.013;

    public path_follower(drivetrain drivetrain){
        this.drivetrain = drivetrain;
        velocity_calculator = new velocity_calculator();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    /* not holding angle */
    public void followTrajectory(curve Traj){
        velocity_calculator.setTrajectory(Traj);
        double t = 0;

        while (t < 0.94 && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            Vector2d transV = velocity_calculator.getTranslationalV(t, dataStorage.RobotPose, dataStorage.DSTelemetry);
            logData(Traj, t);
            drivetrain.applyVectorFieldCentric(transV.rotated(Math.toRadians(90)), 0);
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    /* holding specified angle */
    public void followTrajectory(curve Traj, double angle){
        followTrajectory(Traj, angle, new double[]{}, new int[]{});
    }

    public void followTrajectory(curve Traj, double angle, double[] ts, int[] actions){
        velocity_calculator.setTrajectory(Traj);
        double t = 0;
        int tIndex = 0;

        while (t < 0.94 && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            if (tIndex < ts.length && t >= ts[tIndex])
            {
                module_master.doAction(actions[tIndex]);
                tIndex++;
            }
            Vector2d transV = velocity_calculator.getTranslationalV(t, dataStorage.RobotPose, dataStorage.DSTelemetry);
            double rotation = velocity_calculator.getRotationCustomDirection(angle);
            logData(Traj, t);
            drivetrain.applyVectorFieldCentric(transV.rotated(Math.toRadians(90)), rotation);
            //drivetrain.applyVectorFieldCentric(new Vector2d(0, 0), rotation);
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void followTrajectoryForwards(curve Traj, double[] ts, int[] actions){
        velocity_calculator.setTrajectory(Traj);
        double t = 0;
        int tIndex = 0;

        while (t < 0.94 && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            if (tIndex < ts.length && t >= ts[tIndex])
            {
                module_master.doAction(actions[tIndex]);
                tIndex++;
            }
            Vector2d transV = velocity_calculator.getTranslationalV(t, dataStorage.RobotPose, dataStorage.DSTelemetry);
            double rotation = velocity_calculator.getRotation(t);
            logData(Traj, t);
            drivetrain.applyVectorFieldCentric(transV.rotated(Math.toRadians(90)), rotation);
            //drivetrain.applyVectorFieldCentric(new Vector2d(0, 0), rotation);
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void followTrajectoryForwards(curve Traj, double percentage, double[] ts, int[] actions){
        velocity_calculator.setTrajectory(Traj);
        double t = 0;
        int tIndex = 0;

        while (t < percentage / 100 && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            if (tIndex < ts.length && t >= ts[tIndex])
            {
                module_master.doAction(actions[tIndex]);
                tIndex++;
            }
            Vector2d transV = velocity_calculator.getTranslationalV(t, dataStorage.RobotPose, dataStorage.DSTelemetry);
            double rotation = velocity_calculator.getRotation(t);
            logData(Traj, t);
            drivetrain.applyVectorFieldCentric(transV.rotated(Math.toRadians(90)), rotation);
            //drivetrain.applyVectorFieldCentric(new Vector2d(0, 0), rotation);
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void followTrajectoryForwardsPercentageHypeAngleControl(curve Traj, double percentage, double angle, double[] ts, int[] actions){
        velocity_calculator.setTrajectory(Traj);
        double t = 0;
        int tIndex = 0;

        while (t < 0.94 && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            if (tIndex < ts.length && t >= ts[tIndex])
            {
                module_master.doAction(actions[tIndex]);
                tIndex++;
            }
            Vector2d transV = velocity_calculator.getTranslationalV(t, dataStorage.RobotPose, dataStorage.DSTelemetry);
            double rotation = 0;
            if (t < percentage / 100)
                rotation = velocity_calculator.getRotation(t);
            else
                rotation = velocity_calculator.getRotationCustomDirection(angle);
            logData(Traj, t);
            drivetrain.applyVectorFieldCentric(transV.rotated(Math.toRadians(90)), rotation);
            //drivetrain.applyVectorFieldCentric(new Vector2d(0, 0), rotation);
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void followTrajectoryForwardsPercentageHypeAngleControl(curve Traj, double trajPercentage, double rotPercentage, double angle, double[] ts, int[] actions){
        velocity_calculator.setTrajectory(Traj);
        double t = 0;
        int tIndex = 0;

        while (t < trajPercentage / 100 && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            if (tIndex < ts.length && t >= ts[tIndex])
            {
                module_master.doAction(actions[tIndex]);
                tIndex++;
            }
            Vector2d transV = velocity_calculator.getTranslationalV(t, dataStorage.RobotPose, dataStorage.DSTelemetry);
            double rotation = 0;
            if (t < rotPercentage / 100)
                rotation = velocity_calculator.getRotation(t);
            else
                rotation = velocity_calculator.getRotationCustomDirection(angle);
            logData(Traj, t);
            drivetrain.applyVectorFieldCentric(transV.rotated(Math.toRadians(90)), rotation);
            //drivetrain.applyVectorFieldCentric(new Vector2d(0, 0), rotation);
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void followTrajectoryBackwards(curve Traj, double[] ts, int[] actions){
        velocity_calculator.setTrajectory(Traj);
        double t = 0;
        int tIndex = 0;

        while (t < 0.94 && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            if (tIndex < ts.length && t >= ts[tIndex])
            {
                module_master.doAction(actions[tIndex]);
                tIndex++;
            }
            Vector2d transV = velocity_calculator.getTranslationalV(t, dataStorage.RobotPose, dataStorage.DSTelemetry);
            double rotation = velocity_calculator.getRotationBackwards(t);
            logData(Traj, t);
            drivetrain.applyVectorFieldCentric(transV.rotated(Math.toRadians(90)), rotation);
            //drivetrain.applyVectorFieldCentric(new Vector2d(0, 0), rotation);
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    /* not holding angle */
    public Vector2d followTrajectoryBreak(curve Traj){
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        velocity_calculator.setTrajectory(Traj);
        double t = 0;
        boolean nearToFinal = false;
        Vector2d plannedPos = Traj.getPoint(0);
        double distanceToFinish = 5.;
        Vector2d finish = Traj.getPoint(1.);

        //dataStorage.DSTelemetry.addData("PID", "off");
        while ((!nearToFinal || t < 0.4) && t < 0.9 && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            Vector2d transV = velocity_calculator.getTranslationalV(t, dataStorage.RobotPose, dataStorage.DSTelemetry);
            double rotation = velocity_calculator.getRotation();
            logData(Traj, t);
            drivetrain.applyVectorFieldCentric(transV.div(1/* * transV.norm()*/).rotated(Math.toRadians(90)), rotation);

            plannedPos = dataStorage.RobotPose.plus(dataStorage.RobotVelocity.times(dataStorage.RobotVelocity.norm() / 2. / dataStorage.BrakeAccel));
            if (plannedPos.distTo(finish) < distanceToFinish) nearToFinal = true;
        }

        double x_error = finish.minus(plannedPos).getX();
        double y_error = finish.minus(plannedPos).getY();
        double sum_x_error = 0, sum_y_error = 0;

        //dataStorage.DSTelemetry.addData("PID", "1");

        while (plannedPos.distTo(finish) > 1){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            logData(Traj, t);
            double old_x_error = x_error;
            double old_y_error = y_error;
            x_error = finish.minus(plannedPos).getX();
            y_error = finish.minus(plannedPos).getY();
            sum_x_error += x_error;
            sum_y_error += y_error;
            double p_x_velocity = x_error * p_trans_coef;
            double d_x_velocity = (x_error - old_x_error) * d_trans_coef;
            double i_x_velocity = sum_x_error * i_trans_coef;

            double p_y_velocity = y_error * p_trans_coef;
            double d_y_velocity = (y_error - old_y_error) * d_trans_coef;
            double i_y_velocity = sum_y_error * i_trans_coef;


            drivetrain.applyVectorFieldCentric(new Vector2d(p_x_velocity + d_x_velocity + i_x_velocity, p_y_velocity + d_y_velocity + i_y_velocity).rotated(Math.toRadians(90)), 0);
            plannedPos = dataStorage.RobotPose.plus(dataStorage.RobotVelocity.times(dataStorage.RobotVelocity.norm() / 2. / dataStorage.BrakeAccel));
        }

        drivetrain.applyVector(new Vector2d(0, 0), 0);

        while (dataStorage.RobotVelocity.norm() > 1)
        {
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
        }

        //dataStorage.DSTelemetry.addData("PID", "2");

        goToPos(finish.getX(), finish.getY(), 0);

        drivetrain.applyVector(new Vector2d(0, 0), 0);

        return plannedPos;
    }

    /* holds specified angle */
    public Vector2d followTrajectoryBreak(curve Traj, double angle){
        return followTrajectoryBreak(Traj, angle, new double[]{}, new int[]{});
    }

    public Vector2d followTrajectoryBreak(curve Traj, double angle, double[] ts, int[] actions){
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        velocity_calculator.setTrajectory(Traj);
        double t = 0;
        int tIndex = 0;
        boolean nearToFinal = false;
        Vector2d plannedPos = Traj.getPoint(0);
        double distanceToFinish = 10.;
        Vector2d finish = Traj.getPoint(1.);

        dataStorage.DSTelemetry.addData("PID", "off");

        while ((!nearToFinal || t < 0.4) && t < 0.9 && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            if (tIndex < ts.length && t >= ts[tIndex])
            {
                module_master.doAction(actions[tIndex]);
                tIndex++;
            }
            Vector2d transV = velocity_calculator.getTranslationalV(t, dataStorage.RobotPose, dataStorage.DSTelemetry);
            double rotation = velocity_calculator.getRotationCustomDirection(angle);
            logData(Traj, t);
            drivetrain.applyVectorFieldCentric(transV.div(1/* * transV.norm()*/).rotated(Math.toRadians(90)), rotation);

            plannedPos = dataStorage.RobotPose.plus(dataStorage.RobotVelocity.times(dataStorage.RobotVelocity.norm() / 2. / dataStorage.BrakeAccel));
            if (plannedPos.distTo(finish) < distanceToFinish) nearToFinal = true;
        }

        double x_error = finish.minus(plannedPos).getX();
        double y_error = finish.minus(plannedPos).getY();
        double sum_x_error = 0, sum_y_error = 0;

        /*
        dataStorage.DSTelemetry.addData("PID", "1");


        while (plannedPos.distTo(finish) > 1){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);
            t = velocity_calculator.getCurrentT(dataStorage.RobotPose);
            logData(Traj, t);
            double old_x_error = x_error;
            double old_y_error = y_error;
            x_error = finish.minus(plannedPos).getX();
            y_error = finish.minus(plannedPos).getY();
            sum_x_error += x_error;
            sum_y_error += y_error;
            double p_x_velocity = x_error * p_trans_coef;
            double d_x_velocity = (x_error - old_x_error) * d_trans_coef;
            double i_x_velocity = sum_x_error * i_trans_coef;

            double p_y_velocity = y_error * p_trans_coef;
            double d_y_velocity = (y_error - old_y_error) * d_trans_coef;
            double i_y_velocity = sum_y_error * i_trans_coef;


            drivetrain.applyVectorFieldCentric(new Vector2d(p_x_velocity + d_x_velocity + i_x_velocity, p_y_velocity + d_y_velocity + i_y_velocity).rotated(Math.toRadians(90)), velocity_calculator.getRotationCustomDirection(new Vector2d(1, 0).rotated(angle)));
            plannedPos = dataStorage.RobotPose.plus(dataStorage.RobotVelocity.times(dataStorage.RobotVelocity.norm() / 2. / dataStorage.BrakeAccel));
        }

         */

        //drivetrain.applyVector(new Vector2d(0, 0), 0);

        //while (dataStorage.RobotVelocity.norm() > 1) dataStorage.updateData();

        //dataStorage.DSTelemetry.addData("PID", "2");

        goToPos(finish.getX(), finish.getY(), angle);

        drivetrain.applyVector(new Vector2d(0, 0), 0);

        return plannedPos;
    }

    public void goToPos(double X, double Y, double Heading){
        Pose2d pid;
        Vector2d finish = new Vector2d(X, Y);

        dataStorage.updateData();
        module_master.update(dataStorage.telemetry);
        while((dataStorage.RobotPose.distTo(finish) > 1 || Math.abs(dataStorage.RobotWorldHeading - Heading) > 0.027 || dataStorage.RobotVelocity.norm() > 3) && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);

            pid = velocity_calculator.getPIDpower(finish, Heading);
            //dataStorage.telemetry.addData("HeadingBuffer", velocity_calculator.IHeading);

            drivetrain.applyVectorFieldCentric(pid.vec().rotated(Math.toRadians(90)), pid.getHeading());
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void goToPosWithArm(double X, double Y, double Heading){
        Pose2d pid;
        Vector2d finish = new Vector2d(X, Y);

        dataStorage.updateData();
        module_master.update(dataStorage.telemetry);
        while((dataStorage.RobotPose.distTo(finish) > 1 || Math.abs(dataStorage.RobotWorldHeading - Heading) > 0.06 || dataStorage.RobotVelocity.norm() > 4) && dataStorage.OpMode.opModeIsActive()){
            if (module_master.arm.extensionMotor.getCurrentPosition() < 250) {
                module_master.differential.setPitch(25);
                module_master.differential.update();
            }
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);

            pid = velocity_calculator.getPIDpower(finish, Heading);
            //dataStorage.telemetry.addData("HeadingBuffer", velocity_calculator.IHeading);

            drivetrain.applyVectorFieldCentric(pid.vec().rotated(Math.toRadians(90)), pid.getHeading());
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void goToPosWithArmThirdSample(double X, double Y, double Heading){
        Pose2d pid;
        Vector2d finish = new Vector2d(X, Y);

        dataStorage.updateData();
        module_master.update(dataStorage.telemetry);
        while((dataStorage.RobotPose.distTo(finish) > 1 || Math.abs(dataStorage.RobotWorldHeading - Heading) > 0.027 || dataStorage.RobotVelocity.norm() > 3) && dataStorage.OpMode.opModeIsActive()){
            if (module_master.arm.rotationState == arm.rotation.RESET) {
                module_master.arm.setExtension(arm.extension.YELLOW_3_PRO);
            }
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);

            pid = velocity_calculator.getPIDpower(finish, Heading);
            //dataStorage.telemetry.addData("HeadingBuffer", velocity_calculator.IHeading);

            drivetrain.applyVectorFieldCentric(pid.vec().rotated(Math.toRadians(90)), pid.getHeading());
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void goToPosWithArmToBasket(double X, double Y, double Heading){
        Pose2d pid;
        Vector2d finish = new Vector2d(X, Y);

        dataStorage.updateData();
        module_master.update(dataStorage.telemetry);
        while((dataStorage.RobotPose.distTo(finish) > 3 || Math.abs(dataStorage.RobotWorldHeading - Heading) > 0.14 || dataStorage.RobotVelocity.norm() > 25) && dataStorage.OpMode.opModeIsActive()){
            if (Math.abs(module_master.arm.targetRotationPos - module_master.arm.rotationMotor.getCurrentPosition()) < 480 && module_master.arm.rotationState == arm.rotation.LIFT)
                module_master.arm.setExtension(arm.extension.EXTENDED);
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);

            pid = velocity_calculator.getPIDpower(finish, Heading);
            //dataStorage.telemetry.addData("HeadingBuffer", velocity_calculator.IHeading);

            drivetrain.applyVectorFieldCentric(pid.vec().rotated(Math.toRadians(90)), pid.getHeading());
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void goToPosUnsafe(double X, double Y, double Heading){
        Pose2d pid;
        Vector2d finish = new Vector2d(X, Y);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        dataStorage.updateData();
        module_master.update(dataStorage.telemetry);
        while(((dataStorage.RobotPose.distTo(finish) > 1 || Math.abs(dataStorage.RobotWorldHeading - Heading) > 0.07 || dataStorage.RobotVelocity.norm() > 3) && timer.milliseconds() < 2000) && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);

            pid = velocity_calculator.getPIDpower(finish, Heading);
            //dataStorage.telemetry.addData("HeadingBuffer", velocity_calculator.IHeading);

            drivetrain.applyVectorFieldCentric(pid.vec().rotated(Math.toRadians(90)), pid.getHeading());
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void goToPosVeryUnsafe(double X, double Y, double Heading){
        Pose2d pid;
        Vector2d finish = new Vector2d(X, Y);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        dataStorage.updateData();
        module_master.update(dataStorage.telemetry);
        while(((dataStorage.RobotPose.distTo(finish) > 2.5 || Math.abs(dataStorage.RobotWorldHeading - Heading) > 0.3 || dataStorage.RobotVelocity.norm() > 15) && timer.milliseconds() < 1200) && dataStorage.OpMode.opModeIsActive()){
            dataStorage.updateData();
            module_master.update(dataStorage.telemetry);

            pid = velocity_calculator.getPIDpower(finish, Heading);
            //dataStorage.telemetry.addData("HeadingBuffer", velocity_calculator.IHeading);

            drivetrain.applyVectorFieldCentric(pid.vec().rotated(Math.toRadians(90)), pid.getHeading());
        }
        drivetrain.applyVector(new Vector2d(0, 0), 0);
    }

    public void logData(curve Traj, double t){
        TelemetryPacket packet = new TelemetryPacket(true);
        Canvas fieldOverlay = packet.fieldOverlay();

        painter.prepare(packet, fieldOverlay);

        Vector2d currentT = dataStorage.RobotPose;
        Vector2d velocity = Traj.getFirstDerivative(t);
        Vector2d realVelocity = dataStorage.RobotVelocity;
        Vector2d trans = velocity_calculator.getTranslationalV(t, currentT, dataStorage.DSTelemetry);

        painter.drawPolyLine(Traj.points);
        painter.drawPoint(currentT.getX(), currentT.getY(), "purple");
        painter.drawRobot(currentT.getX(), currentT.getY(), dataStorage.RobotWorldHeading, "magenta");
        painter.drawPolyLine(dataStorage.poseHistory.toArray(new Vector2d[0]), "blue");
        painter.drawVector(currentT, realVelocity.times(20).plus(currentT), "red");
        painter.drawVector(currentT, velocity.times(20).plus(currentT), "orange");
        painter.drawVector(currentT, velocity_calculator.getCentripetalVelocity().times(20).plus(currentT), "lime");
        painter.drawVector(currentT, velocity_calculator.getErrorVelocity(dataStorage.DSTelemetry).times(20).plus(currentT), "cyan");
        painter.drawVector(currentT, trans.times(20).plus(currentT), "purple");
        painter.drawGround(-24, 24, "black");
        painter.drawGround(0, 48, "red");
        painter.drawGround(24, 24, "green");

        //packet.put("radius", Traj.getRadius(t));
        //packet.put("transV", trans.toString());
        //packet.put("realVel", realVelocity.toString());
        //packet.put("angle", dataStorage.RobotWorldHeading);
        packet.put("cycle time", dataStorage.timer.milliseconds());
        packet.put("t", t);
        //packet.put("x", dataStorage.RobotPose.getX());
        //packet.put("heading error", dataStorage.RobotWorldHeading);
        packet.put("real velocity", realVelocity.norm());
        packet.put("target velocity", velocity_calculator.targetVelocity);
        packet.put("closest turn start", velocity_calculator.findClosestTurn()[0]);
        packet.put("closest turn end", velocity_calculator.findClosestTurn()[1]);
        packet.put("state", velocity_calculator.state);
        packet.put("radius", velocity_calculator.getTurnRadius(velocity_calculator.findClosestTurn()[0], velocity_calculator.findClosestTurn()[1]));

        dashboard.sendTelemetryPacket(packet);
    }
}
