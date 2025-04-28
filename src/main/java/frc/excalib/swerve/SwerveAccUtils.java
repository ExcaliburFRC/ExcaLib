package frc.excalib.swerve;

import frc.excalib.control.math.Vector2D;

import static frc.excalib.control.math.MathUtils.minSize;
import static frc.robot.SwerveConstants.*;

public class SwerveAccUtils {
    private static final double CYCLE_TIME = 0.02;

    /**
     * A function to get the translational velocity setpoint
     *
     * @param velocitySetPoint wanted velocity setpoint
     * @return translational velocity setpoint
     */
    public static Vector2D getSmartTranslationalVelocitySetPoint(Vector2D currentVel, Vector2D velocitySetPoint) {
        Vector2D deltaVelocity = velocitySetPoint.plus(
                currentVel.mul(-1));
        Vector2D actualDeltaVelocity = applyAccelerationLimits(currentVel, deltaVelocity);
        return currentVel.plus(actualDeltaVelocity);
    }

    /**
     * A function to apply the acceleration limits
     *
     * @param velocityError wanted velocity
     * @return velocity limited by acceleration limits
     */
    private static Vector2D applyAccelerationLimits(Vector2D currentVel, Vector2D velocityError) {
        Vector2D wantedAcceleration = velocityError.mul(1 / CYCLE_TIME);

//        wantedAcceleration = applyForwardLimit(currentVel, wantedAcceleration);
//        wantedAcceleration = applyTiltLimit(wantedAcceleration);
        wantedAcceleration = applySkidLimit(wantedAcceleration);

        return wantedAcceleration.mul(CYCLE_TIME);
    }

    /**
     * A function to apply the forward acceleration limit
     *
     * @param wantedAcceleration the wanted acceleration
     * @return wanted acceleration limited by forward acceleration limit
     */
    private static Vector2D applyForwardLimit(Vector2D currentVel, Vector2D wantedAcceleration) {
        double maxAcceleration =
                MAX_FORWARD_ACC *
                        (1 - (currentVel.getDistance() / MAX_VEL));

        wantedAcceleration = wantedAcceleration.limit(new Vector2D(maxAcceleration, currentVel.getDirection()));
        return wantedAcceleration;
    }

    /**
     * A function to apply the tilt acceleration limit
     *
     * @param wantedAcceleration the wanted acceleration
     * @return wanted acceleration limited by tilt acceleration limit
     */
    private static Vector2D applyTiltLimit(Vector2D wantedAcceleration) {
        double frontAcceleration = minSize(wantedAcceleration.getX(), MAX_FRONT_ACC);
        double sideAcceleration = minSize(wantedAcceleration.getY(), MAX_SIDE_ACC);
        return new Vector2D(frontAcceleration, sideAcceleration);
    }

    /**
     * A function to apply the skid acceleration limit
     *
     * @param wantedAcceleration the wanted acceleration
     * @return wanted acceleration limited by skid acceleration limit
     */
    private static Vector2D applySkidLimit(Vector2D wantedAcceleration) {
        return new Vector2D(
                Math.min(wantedAcceleration.getDistance(), MAX_SKID_ACC),
                wantedAcceleration.getDirection()
        );
    }
}
