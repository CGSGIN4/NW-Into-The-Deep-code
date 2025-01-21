package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public enum bezierCurves {;
    private static Vector2d lerp(Vector2d p1, Vector2d p2, double t){
        return p1.times(1-t).plus(p2.times(t));
    }
    /*public static Vector2d[] buildCubicBernstein(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4, int iter){
        Vector2d[] points = new Vector2d[iter];
        double t = 0.0;
        double step = 1.0 / (iter - 1);
        for (int i = 0; i < iter; i++, t += step){
            double t2 = t * t;
            double t3 = t2 * t;
            points[i] = p1.times(-t3 + 3*t2 - 3*t + 1).plus(p2.times(3*t3 - 6*t2 + 3*t)).plus(p3.times(-3*t3 + 3*t2)).plus(p4.times(t3));
        }
        return points;
    }*/

    public static Vector2d[] buildCubicBernstein(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4, int iter){
        Vector2d[] points = new Vector2d[iter + 1];
        double t = 0.0;
        double step = 1.0 / (iter);
        for (int i = 0; i <= iter; i++, t += step){
            double t2 = t * t;
            double t3 = t2 * t;
            points[i] = p1.times(-t3 + 3*t2 - 3*t + 1).plus(p2.times(3*t3 - 6*t2 + 3*t)).plus(p3.times(-3*t3 + 3*t2)).plus(p4.times(t3));
        }
        return points;
    }
    public static Vector2d[] buildCubicBernstein(Vector2d[] points, int iter) {
        if (points.length != 4){
            return new Vector2d[0];
        }
        return buildCubicBernstein(points[0], points[1], points[2], points[3], iter);
    }

    public static Vector2d[] buildCubicCasteljau(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4, int iter){
        Vector2d[] points = new Vector2d[iter];
        double t = 0.0;
        double step = 1.0 / (iter - 1);

        for (int i = 0; i < iter; i++, t += step) {
            Vector2d A = lerp(p1, p2, t);
            Vector2d B = lerp(p2, p3, t);
            Vector2d C = lerp(p3, p4, t);
            Vector2d D = lerp(A, B, t);
            Vector2d E = lerp(B, C, t);
            points[i] = lerp(D, E, t);
        }
        return points;
    }

    public static Vector2d[] buildCubicCasteljau(Vector2d[] points, int iter){
        if (points.length != 4)
            return new Vector2d[0];
        return buildCubicCasteljau(points[0], points[1], points[2], points[3], iter);
    }
    public static Vector2d[] buildCubicPolynomial(Vector2d p0, Vector2d p1, Vector2d p2, Vector2d p3, int iter){
        Vector2d[] points = new Vector2d[iter];
        double t = 0.0;
        double t2;
        double t3;
        double step = 1.0 / (iter - 1);

        for (int i = 0; i < iter; i++, t += step) {
            t2 = t * t;
            t3 = t2 * t;
            Vector2d P = p0
                    .plus((p0.times(-3).plus(p1.times(3))).times(t))
                    .plus((p0.times(3).plus(p1.times(-6)).plus(p2.times(3))).times(t2))
                    .plus((p0.times(-1).plus(p1.times(3)).plus(p2.times(-3)).plus(p3)).times(t3));
            points[i] = P;
        }
        return points;
    }
    public static Vector2d[] buildCubicPolynomial(Vector2d[] points, int iter){
        if (points.length != 4)
            return new Vector2d[0];
        return buildCubicPolynomial(points[0], points[1], points[2], points[3], iter);
    }
    /*public static Vector2d getDerivative(double t){
        Vector2d P = p0.times(-3).plus(p1.times(3))
                .plus((p0.times(-3).plus(p1.times(3))).times(t))
                .plus((p0.times(3).plus(p1.times(-6)).plus(p2.times(3))).times(t*t))
                .plus((p0.times(-1).plus(p1.times(3)).plus(p2.times(-3)).plus(p3)).times(t*t*t));
        return P;
    }*/
}
