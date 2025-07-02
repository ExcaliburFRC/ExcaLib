package frc.excalib.swerve.swerve_utils;

import frc.excalib.control.math.Vector2D;

import static frc.excalib.control.math.MathUtils.minSize;

/**
 * A util class that applies acceleration limits on the swerve.
 */
public class SwerveAccUtils {
    private static final double CYCLE_TIME = 0.02;
    private static SwerveSpecs kSpecs;

    /**
     * A function that sets the swerve specs.
     *
     * @param specs The swerve specs
     */
    public static void setSwerveSpecs(SwerveSpecs specs) {
        kSpecs = specs;
    }

    /**
     * A function to get the translational velocity setpoint.
     *
     * @param currentVel       the current velocity of the swerve
     * @param velocitySetpoint wanted velocity setpoint
     * @param allLimits        whether to use all limits or only the skid limit
     * @return translational velocity setpoint with acceleration limits
     */
    public static Vector2D getSmartTranslationalVelocitySetpoint(Vector2D currentVel, Vector2D velocitySetpoint, boolean allLimits) {
        Vector2D deltaVelocity = velocitySetpoint.plus(
                currentVel.mul(-1));
        Vector2D actualDeltaVelocity = applyAccelerationLimits(currentVel, deltaVelocity, allLimits);
        return currentVel.plus(actualDeltaVelocity);
    }

    /**
     * A function to apply the acceleration limits
     *
     * @param currentVel    the current velocity of the swerve
     * @param velocityError wanted velocity
     * @param allLimits     whether to use all limits or only the skid limit
     * @return velocity limited by acceleration limits
     */
    private static Vector2D applyAccelerationLimits(Vector2D currentVel, Vector2D velocityError, boolean allLimits) {
        Vector2D wantedAcceleration = velocityError.mul(1 / CYCLE_TIME);

        if (allLimits) {
            wantedAcceleration = applyForwardLimit(currentVel, wantedAcceleration);
            wantedAcceleration = applyTiltLimit(wantedAcceleration);
        }
        wantedAcceleration = applySkidLimit(wantedAcceleration);

        return wantedAcceleration.mul(CYCLE_TIME);
    }

    /**
     * A function to apply the forward acceleration limit
     *
     * @param currentVel         the current velocity of the swerve
     * @param wantedAcceleration the wanted acceleration
     * @return wanted acceleration limited by forward acceleration limit
     */
    private static Vector2D applyForwardLimit(Vector2D currentVel, Vector2D wantedAcceleration) {
        // TODO: check & debug
        double maxAcceleration =
                kSpecs.maxForwardAcc() *
                        (1 - (currentVel.getDistance() / kSpecs.maxVelocity()));

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
        // TODO: check & debug
        double frontAcceleration = minSize(wantedAcceleration.getX(), kSpecs.maxFrontAcc().getAsDouble());
        double sideAcceleration = minSize(wantedAcceleration.getY(), kSpecs.maxSideAcc().getAsDouble());
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
                Math.min(wantedAcceleration.getDistance(), kSpecs.maxAcc()),
                wantedAcceleration.getDirection()
        );
    }
}
