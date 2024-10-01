package excalib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import excalib.math.Vector2D;
import excalib.math.control.Gains;
import excalib.math.control.motor.Motor;
import excalib.mechanisms.fly_wheel.FlyWheel;
import excalib.mechanisms.turret.Turret;

import java.util.function.DoubleSupplier;

import static excalib.swerve.Constants.MAX_MODULE_VELOCITY;
import static excalib.swerve.Constants.WHEEL_RADIUS;

/**
 * A class representing a swerve module
 *
 * @authors Yoav Cohen & Itay Keller
 */
public class SwerveModule {
    private final FlyWheel m_driveWheel;
    private final Turret m_turret;
    private final Translation2d m_MODULE_LOCATION;

    /**
     * A constructor for the SwerveModule
     *
     * @param driveMotor    the drive motor
     * @param rotationMotor the rotation motor
     * @param angleGains    the rotation gains
     * @param PIDtolerance  the tolerance of the PID for the rotating motor
     */
    public SwerveModule(Motor driveMotor, Motor rotationMotor, Gains angleGains, double PIDtolerance, Translation2d moduleLocation, DoubleSupplier angleSupplier) {
        m_driveWheel = new FlyWheel(driveMotor, WHEEL_RADIUS);
        m_turret = new Turret(rotationMotor, null, angleGains, PIDtolerance, angleSupplier);
        m_MODULE_LOCATION = moduleLocation;
    }

    double getVelocityRatioLimit(Vector2D translationVelocity, double omegaRadPerSec) {
        Vector2D rotationVector = new Vector2D(
                omegaRadPerSec,
                m_MODULE_LOCATION.getAngle().rotateBy(new Rotation2d(Math.PI / 2))
        );
        Vector2D sigmaVel = translationVelocity.plus(rotationVector);
        return sigmaVel.getDistance() / MAX_MODULE_VELOCITY;
    }

    void setVelocity(
            Vector2D translationVelocity,
            double omegaRadPerSec,
            double velocityRatioLimit) {
        Vector2D rotationVector = new Vector2D(
                omegaRadPerSec,
                m_MODULE_LOCATION.getAngle().rotateBy(new Rotation2d(Math.PI / 2))
        );
        Vector2D sigmaVel = translationVelocity.plus(rotationVector);
        sigmaVel = sigmaVel.mul(velocityRatioLimit);
        m_driveWheel.setOutput(sigmaVel.getDistance(), FlyWheel.Unit.MPS);
        m_turret.setPosition(sigmaVel.getDirection(), Swerve.getInstance());
    }

    /**
     * A function that stops the Module
     */
    void stopModule() {
        m_driveWheel.stopWheel();
    }

    /**
     * A function to get the module's velocity
     *
     * @return a Vector2d that represents the velocity
     */
    Vector2D getVelocity() {
        return new Vector2D(m_driveWheel.getVelocityMPS(), getPosition());
    }

    Rotation2d getPosition() {
        return m_turret.getPosition();
    }
}
