package frc.excalib.control.math;

import edu.wpi.first.math.geometry.Translation2d;

public class Line {
    public final double a, b, c;

    public Line(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public Translation2d findIntersection(Line other) {
        return new Translation2d(
                (other.b * c - b * other.c) / (b * other.a - other.b * a),
                (other.a * c - a * other.c) / (a * other.b - other.a * b)
        );
    }
    @Override
    public String toString(){
        return "a: "+a+", b: "+b+", c: "+c;
    }
}