package excalib.swerve;

import excalib.math.Vector2D;

public record ModulesHolder(
        SwerveModule frontLeft,
        SwerveModule frontRight,
        SwerveModule backLeft,
        SwerveModule backRight) {

    /**
     * A function to set the velocities of the modules
     *
     * @param translationVelocity the modules' velocity
     * @param omegaRadPerSec the rotation speed of the modules
     */
    void setVelocities(Vector2D translationVelocity, double omegaRadPerSec) {
        double velocityRatioLimit =
                Math.min(
                        Math.min(
                                frontLeft.getVelocityRatioLimit(translationVelocity, omegaRadPerSec),
                                frontRight.getVelocityRatioLimit(translationVelocity, omegaRadPerSec)),
                        Math.min(
                                backLeft.getVelocityRatioLimit(translationVelocity, omegaRadPerSec),
                                backRight.getVelocityRatioLimit(translationVelocity, omegaRadPerSec)));
        velocityRatioLimit = Math.min(1, velocityRatioLimit);
        frontLeft.setVelocity(translationVelocity, omegaRadPerSec, velocityRatioLimit);
        frontRight.setVelocity(translationVelocity, omegaRadPerSec, velocityRatioLimit);
        backLeft.setVelocity(translationVelocity, omegaRadPerSec, velocityRatioLimit);
        backRight.setVelocity(translationVelocity, omegaRadPerSec, velocityRatioLimit);
    }

    /**
     * A function that stops the modules
     */
    void stop() {
        frontLeft.stopModule();
        frontRight.stopModule();
        backLeft.stopModule();
        frontRight.stopModule();
    }

    /**
     * A function to get the robot's velocity
     *
     * @return a Vector2d that represents the robot's velocity
     */
    Vector2D getVelocity() {
        Vector2D velocity = frontLeft.getVelocity();
        velocity = velocity.plus(frontRight.getVelocity());
        velocity = velocity.plus(backLeft.getVelocity());
        velocity = velocity.plus(backRight.getVelocity());
        return velocity.mul(0.25);
    }
}
