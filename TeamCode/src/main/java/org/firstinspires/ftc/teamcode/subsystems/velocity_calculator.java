package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.math.normalizeAngle.normalizeAngle;
import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.round;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.calculator;
import org.firstinspires.ftc.teamcode.math.curve;
import org.firstinspires.ftc.teamcode.math.line;
import org.firstinspires.ftc.teamcode.utils.ring_buffer;

public class velocity_calculator {
    curve trajectory;
    follower_state state = follower_state.STRAIGHT;
    double t;
    double CENTRIPETAL_ACCEL_KOEF = 0.65/*0.59*/;

    double P_ROTATION_COEF = 0.34 /*0.2*/;

    double D_ROTATION_COEF = 0.11 /*0.4*/;

    double I_ROTATION_COEF = 0.15;//3;
    double velocityModule;

    double oldTargetHeading;
    double step;
    double minDistance;

    double targetVelocity;

    double perpError = 0;
    double perpErrorOld = 0;
    double perpErrorBuffer = 0;

    double speedError;
    double speedErrorOld = 0;

    double PPERPENDICULAR = /*0.33, 0.38*/ 0.3;
    double DPERPENDICULAR = -0.18;
    double IPERPENDICULAR = 0;

    double PLINEAR = 0.03;
    double DLINEAR = 0.1;

    public double IHeading = 0;
    Vector2d robotVelocity;
    public Vector2d accel;
    public Vector2d velocity;
    double closest_turn_old;

    double p_rotation_coef = 3.;
    double p_trans_coef = 0.057;
    double d_trans_coef = 0.33;
    double i_trans_coef = 0.01;

    double x_error = 0;
    double y_error = 0;
    double sum_x_error = 0, sum_y_error = 0;

    public void setTrajectory(curve trajectory){
        this.trajectory = trajectory;
        this.t = 0;
        this.oldTargetHeading = trajectory.getFirstDerivative(this.t).angle();
    }
    public Vector2d getTranslationalV(double t, Vector2d robotPos, Telemetry telemetry){
        this.t = t;
        this.velocity = trajectory.getFirstDerivative(t);
        this.accel = trajectory.getSecondDerivative(t).projectOnto(velocity.rotated(PI/2));
        this.velocityModule = velocity.norm();
        this.robotVelocity = dataStorage.RobotVelocity;

        Vector2d errorVel = getErrorVelocity(telemetry);
        Vector2d V = getV();

        /* scaling coefficient for pathing power */
        double a;

        if (errorVel.norm() == 0)
            a = 1. / V.norm();
        else
        {
            double Alpha = errorVel.angleBetween(V);
            double Betta = asin(sin(Alpha) * errorVel.norm());
            double Gamma = PI - Alpha - Betta;

            a = sin(Gamma) / (sin(Alpha) * V.norm());
        }

        double min_radius = 10000.;
        for (int i = 4; i <= 10; i++) min_radius = min(min_radius, trajectory.getRadius(t + step * i));

        double[] closest_turn = findClosestTurn();
        switch (state)
        {
            case STRAIGHT:
                if (getClosestTurnBrakePath() >= trajectory.getPoint(closest_turn[0]).minus(robotPos).norm() && closest_turn[0] <= 1.)
                    state = follower_state.SLOWING_DOWN;
                if (t >= closest_turn[0])
                    state = follower_state.BEFORE_APEKS;
                break;
            case SLOWING_DOWN:
                if (robotVelocity.norm() < targetVelocity)
                    state = follower_state.STRAIGHT;
                if (t >= closest_turn[0])
                    state = follower_state.BEFORE_APEKS;
                return new Vector2d(0, 0);//errorVel.times(0.2);
            case BEFORE_APEKS:
                a *= Math.min(getMultiplierFromRadius((getTurnRadius(closest_turn[0], closest_turn[1]))), 1);

                if (t > closest_turn[0] + (closest_turn[1] - closest_turn[0]) / 1.5)
                    state = follower_state.AFTER_APEKS;

                dataStorage.DSTelemetry.addData("V", V.toString());
                dataStorage.DSTelemetry.update();
                closest_turn_old = closest_turn[0];
                return V.times(a).plus(errorVel);
            case AFTER_APEKS:
                trajectory.turn_starts.remove(closest_turn[0]);
                trajectory.turn_ends.remove(closest_turn[1]);
                state = follower_state.STRAIGHT;
                if (t >= closest_turn[1] || closest_turn[0] != closest_turn_old) {
                    closest_turn_old = closest_turn[0];
                    state = follower_state.STRAIGHT;
                }
                break;
        }

        V = V.times(a).plus(errorVel);
        return V;
    }

    public double getRotation(double t){
        this.velocity = this.trajectory.getFirstDerivative(t);
        return (dataStorage.RobotWorldHeading - this.velocity.angle()) * P_ROTATION_COEF;
    }

    public double getRotation(){
        double currentTargetHeading = 0;//this.velocity.angle();
        double errorHeading = normalizeAngle(currentTargetHeading - dataStorage.RobotWorldHeading);
        double errorOldHeading = dataStorage.OldRobotWorldHeading - this.oldTargetHeading;

        double DHeading = errorHeading - errorOldHeading;

        if (Math.abs(errorHeading) <= 0.3)
            IHeading += errorHeading;
        if (errorHeading * errorOldHeading < 0)
            IHeading = 0;

        this.oldTargetHeading = currentTargetHeading;
        return errorHeading * P_ROTATION_COEF + DHeading * D_ROTATION_COEF + this.IHeading * I_ROTATION_COEF;
    }

    public double getRotationCustomDirection(Vector2d direction){
        double currentTargetHeading = normalizeAngle(direction.angle());
        double errorHeading = normalizeAngle(currentTargetHeading - dataStorage.RobotWorldHeading);
        double errorOldHeading = normalizeAngle(dataStorage.OldRobotWorldHeading - this.oldTargetHeading);

        if (currentTargetHeading != oldTargetHeading)
            IHeading = 0;
        double DHeading = errorHeading - errorOldHeading;

        if (Math.abs(errorHeading) <= 0.45)
            IHeading += errorHeading;
        if (Math.abs(errorHeading) < 0.05)
            IHeading = 0;

        this.oldTargetHeading = currentTargetHeading;
        return errorHeading * P_ROTATION_COEF + DHeading * D_ROTATION_COEF + this.IHeading * I_ROTATION_COEF;
    }

    public double getRotationCustomDirection(double angle){
        double currentTargetHeading = normalizeAngle(angle);
        double errorHeading = normalizeAngle(currentTargetHeading - dataStorage.RobotWorldHeading);
        double errorOldHeading = dataStorage.OldRobotWorldHeading - this.oldTargetHeading;

        double DHeading = errorHeading - errorOldHeading;

        if (Math.abs(errorHeading) <= 0.3)
            IHeading += errorHeading;
        if (errorHeading * errorOldHeading < 0)
            IHeading = 0;

        this.oldTargetHeading = currentTargetHeading;
        return errorHeading * P_ROTATION_COEF + DHeading * D_ROTATION_COEF + this.IHeading * I_ROTATION_COEF;
    }

    public double getCurrentT(Vector2d robotPos){
        double minDistance = 1000;
        double minT = 0;
        double currentT = 0;
        this.step = 1.0 / (trajectory.points.length - 1);

        for (Vector2d i : trajectory.points){
            if (robotPos.distTo(i) <= minDistance){
                minDistance = robotPos.distTo(i);
                minT = currentT;
            }
            currentT += step;
        }
        this.minDistance = minDistance;
        return minT;
    }
    public Vector2d getErrorVelocity(Telemetry telemetry){
        perpError = trajectory.getPoint(t).minus(dataStorage.RobotPose).projectOnto(accel).norm();
        double perpP = perpError;
        double perpD = perpError - perpErrorOld;
        if (Math.abs(perpError) <= 2)
            perpErrorBuffer += perpError;
        if (perpError * perpErrorOld < 0)
            perpErrorBuffer = 0;

        double Radius = this.velocityModule * this.velocityModule / accel.norm();
        Vector2d Center = trajectory.points[(int) round(t * (trajectory.points.length - 1))].plus(accel.div(accel.norm()).times(Radius));
        if (dataStorage.RobotPose.distTo(Center) < Radius)
            return accel.div(accel.norm()).times(max(perpP * -PPERPENDICULAR + perpD * -DPERPENDICULAR + perpErrorBuffer * -IPERPENDICULAR, -1));
        else
            return accel.div(accel.norm()).times(min(perpP * PPERPENDICULAR + perpD * DPERPENDICULAR + perpErrorBuffer * IPERPENDICULAR, 1));
    }

    public Vector2d getCentripetalVelocity(){
        //return accel.times((robotVelocity.norm() * robotVelocity.norm()) / (velocityModule * velocityModule)).times(centripetalAccelCoef); TODO: before pedro
        //return trajectory.getSecondDerivative(t + step * 1).times((robotVelocity.norm() * robotVelocity.norm()) / (velocityModule * velocityModule)).times(centripetalAccelCoef); TODO: get from future
        if (robotVelocity.angleBetween(accel) < Math.toRadians(90))
            return accel.times((pow(robotVelocity.dot(velocity.div(velocity.norm())), 2)) / (velocityModule * velocityModule)).times(CENTRIPETAL_ACCEL_KOEF);
        else
            return accel.times((robotVelocity.norm() * robotVelocity.norm()) / (velocityModule * velocityModule)).times(CENTRIPETAL_ACCEL_KOEF);
        /*return accel.times((2 * robotVelocity.norm() * robotVelocity.norm() - (pow(robotVelocity.dot(velocity.div(velocity.norm())), 2))) / (velocityModule * velocityModule)).times(CENTRIPETAL_ACCEL_KOEF);*/

    }

    @Deprecated
    public Vector2d getSpeedCorrectingVelocity(Vector2d V){
        this.speedError = this.targetVelocity - this.robotVelocity.norm();
        Vector2d finalV = V.div(V.norm()).times(PLINEAR * speedError + DLINEAR * (speedError - speedErrorOld));
        this.speedErrorOld = this.speedError;
        return finalV;
    }

    private double getMultiplierFromRadius(double radius){
        double r2 = radius * radius;
        double r3 = radius * radius * radius;
        //return 0.00000147 * r3 + 0.0001 * r2 + 0.0004 * radius + 0.2989;
        //return 0.0000051573 * r3 - 0.0001488814 * r2 + 0.0020886512 * radius + 0.2966786459;
        //return 0.0000053782 * r3 - 0.0001947513 * r2 +0.0041048603 * radius + 0.2931198344;
        //return 0.0002193633 * r2 - 0.0014049996 * radius + 0.3020281063;
        return -0.0000021394 * r3 + 0.0003362747 * r2 - 0.0008929676 * radius + 0.3784505068; //0.2484505068
    }

    private double getTargetVelocityFromRadius(double radius){
        return 25;
    }

    private Vector2d getV(){
        return this.velocity.plus(getCentripetalVelocity());
    }

    public double getTurnRadius(double start, double end){
        double Theta = trajectory.getFirstDerivative(start).angleBetween(trajectory.getFirstDerivative(end));
        double K = 0;
        double t = start;
        while (t < end){
            K += Math.abs(trajectory.getPoint(t + step).minus(trajectory.getPoint(t)).norm());
            t += step;
        }
        return K / Theta;
    }

    public double[] findClosestTurn(){
        double min = 1.1;
        double end = 1.1;
        double start = 1.1;
        for (double point : trajectory.turn_starts){
            if (Math.abs(point - t) < min && (double) trajectory.turn_ends.toArray()[trajectory.turn_starts.indexOf(point)] > t){
                min = Math.abs(point - t);
                start = point;
                end = (double) trajectory.turn_ends.toArray()[trajectory.turn_starts.indexOf(start)];
            }
        }
        for (double point : trajectory.turn_ends){
            if (Math.abs(point - t) < min && point > t){
                min = Math.abs(point - t);
                end = point;
                start = (double) trajectory.turn_starts.toArray()[trajectory.turn_ends.indexOf(end)];
            }
        }

        return new double[] {start, end};
    }

    public double getClosestTurnBrakePath(){
        double[] turn = findClosestTurn();
        this.targetVelocity = getTargetVelocityFromRadius(getTurnRadius(turn[0], turn[1]));
        return (robotVelocity.norm() * robotVelocity.norm() - targetVelocity * targetVelocity) / (2 * dataStorage.BrakeAccel);
    }

    public Pose2d getPIDpower(Vector2d finish, double angle){
        dataStorage.updateData();

        double old_x_error = x_error;
        double old_y_error = y_error;
        x_error = finish.minus(dataStorage.RobotPose).getX();
        y_error = finish.minus(dataStorage.RobotPose).getY();

        dataStorage.DSTelemetry.addData("xerr", x_error);
        dataStorage.DSTelemetry.addData("yerr", y_error);
        dataStorage.DSTelemetry.update();

        if (x_error < 3)
            sum_x_error += x_error;

        if (y_error < 3)
            sum_y_error += y_error;

        if (x_error * old_x_error < 0)
            sum_x_error = 0;
        if (y_error * old_y_error < 0)
            sum_y_error = 0;

        double p_x_velocity = x_error * p_trans_coef;
        double d_x_velocity = (x_error - old_x_error) * d_trans_coef;
        double i_x_velocity = sum_x_error * i_trans_coef;

        double p_y_velocity = y_error * p_trans_coef;
        double d_y_velocity = (y_error - old_y_error) * d_trans_coef;
        double i_y_velocity = sum_y_error * i_trans_coef;

        double rotation = getRotationCustomDirection(new Vector2d(1, 0).rotated(angle));// * p_rotation_coef;

        return new Pose2d(p_x_velocity + d_x_velocity + i_x_velocity, p_y_velocity + d_y_velocity + i_y_velocity, rotation);
    }

    /* TELEOP SECTION */
    public Vector2d getFinalTeleopForce(Vector2d input, ring_buffer<Vector2d> poseHistory) {
        Vector2d[] points = new Vector2d[poseHistory.getCapacity()];
        double[] firstDerivatives = new double[poseHistory.getCapacity() - 1];
        double[] secondDerivatives = new double[poseHistory.getCapacity() - 2];

        for (int i = 0; i < poseHistory.getCapacity(); i++)
            points[i] = poseHistory.get();

        for (int i = 0; i < poseHistory.getCapacity() - 1; i++) {
            firstDerivatives[i] = getFirstDerivative(points[i + 1], points[i]);
            if (firstDerivatives[i] == 0)
                return input;
        }

        for (int i = 0; i < poseHistory.getCapacity() - 2; i++)
            secondDerivatives[i] = firstDerivatives[i + 1] - firstDerivatives[i];

        double radius = (Math.pow(1 + Math.pow(firstDerivatives[poseHistory.getCapacity() - 2], 2), 3.0/2.0)) / secondDerivatives[0];

        Vector2d realVel = dataStorage.RobotVelocity;
        Vector2d centripetalDir = realVel.rotated(Math.toRadians(90)).div(realVel.norm());

        dataStorage.DSTelemetry.addData("radius", radius);
        dataStorage.DSTelemetry.update();

        if (calculator.findRaySegmentIntersection(points[poseHistory.getCapacity() - 1], centripetalDir.angle(), new Vector2d(points[poseHistory.getCapacity() - 3].getX() * 1000000, points[poseHistory.getCapacity() - 3].getY() * 1000000), new Vector2d(points[poseHistory.getCapacity() - 2].getX() * 1000000, points[poseHistory.getCapacity() - 2].getY() * 1000000)) == null)
            centripetalDir = realVel.rotated(Math.toRadians(-90)).div(realVel.norm());

        Vector2d centripetalVel = centripetalDir.times(realVel.norm() * realVel.norm() / radius * CENTRIPETAL_ACCEL_KOEF);
        Vector2d power = centripetalVel.plus(input);
        power = power.div(power.norm()).times(input.norm());

        return power;
    }

    public double getFirstDerivative(Vector2d p1, Vector2d p2){
        if (p1.getX() != p2.getX())
            return (p1.getY() - p2.getY()) / (p1.getX() - p2.getX());
        return 0;
    }
}