package frc.excalib.control.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class MathUtils {
    /**
     * A function that checks the minimum value in abs
     *
     * @param sizeLimit max size
     * @return minimum double value
     */

    public static double minSize(double val, double sizeLimit) {
        return Math.min(sizeLimit, Math.abs(val)) * Math.signum(val);
    }

    public static double limitTo(double limit, double value) {
        if ((limit > 0 && limit < value) || (limit < 0 && limit > value)) {
            return limit;
        }
        return value;
    }
    public static double getPosesTangentAngle(Translation2d place1, Translation2d place2){
        double angle = Math.atan2(place2.getY() - place1.getY(), place1.getX() - place2.getX());
        if (place1.getY() > place2.getY()) angle += Math.PI;
        return angle;
    }}
