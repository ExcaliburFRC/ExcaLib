package frc.excalib.control.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Circle {
    public final double r;
    public final Translation2d center;

    public Circle(double a, double b, double r) {
        this.center = new Translation2d(a, b);
        this.r = r;
    }

    public Line getTangent(Translation2d pointOnCircle) {
        return new Line(
                pointOnCircle.getX() - center.getX(),
                pointOnCircle.getY() - center.getY(),
                -center.getX() * (pointOnCircle.getX() - center.getX())
                        - center.getY() * (pointOnCircle.getY() - center.getY()) -
                        this.r * this.r
        );
    }

    public Line[] getTangents(Translation2d point) {
        if (point.getDistance(this.center) < this.r) {
            return new Line[0];
        }else if (point.getDistance(this.center) == this.r) {
            return new Line[]{
                    getTangent(point)
            };
        }
        double centersDistance = this.center.getDistance(point);
        double newRad = Math.sqrt(Math.pow(centersDistance, 2) - Math.pow(this.r, 2));
        Circle newCircle = new Circle(point.getX(), point.getY(), newRad);
        Translation2d[] intersections = getInterSections(newCircle);
        Translation2d firstTanPoint = intersections[0];
        Translation2d secondTanPoint = intersections[1];


        return new Line[]{
                getTangent(firstTanPoint),
                getTangent(secondTanPoint)
        };
    }

    public Translation2d[] getInterSections(Circle other) {

        if (other.center.getDistance(this.center) > other.r + r) return new Translation2d[0];
        if (other.center.getDistance(this.center) < Math.abs(other.r - this.r)) return new Translation2d[0];

        if (other.center.getDistance(this.center) == other.r + r) {
            return new Translation2d[]{
                    this.center.plus(new Translation2d(this.r, other.center.minus(this.center).getAngle()))
            };
        }
        if (other.center.getDistance(this.center) < Math.abs(other.r - this.r)) {
            return new Translation2d[]{//check
                    this.center.plus(new Translation2d(this.r, other.center.minus(this.center).getAngle()))
            };
        }
        Rotation2d alpha = new Rotation2d(Math.acos(
                (Math.pow(other.r, 2) - Math.pow(this.r, 2) - Math.pow(this.center.getDistance(other.center), 2))
                        / (-2 * this.center.getDistance(other.center) * this.r)
        ));
        return new Translation2d[]{
                this.center.plus(new Translation2d(this.r, other.center.minus(this.center).getAngle().plus(alpha))),
                this.center.plus(new Translation2d(this.r, other.center.minus(this.center).getAngle().minus(alpha)))
        };
    }
}