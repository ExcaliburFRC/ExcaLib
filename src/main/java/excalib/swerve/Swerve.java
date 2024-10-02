package excalib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import excalib.math.Vector2D;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
/**
 * A class representing a swerve subsystem
 *
 * @author Yoav Cohen & Itay Keller
 */
public class Swerve extends SubsystemBase {
    private final double CYCLE_TIME = 1 / 50.0;
    static Swerve m_instance;
    private final SwerveConfigurator.KinematicsConfig m_kinematicsConfig;
    private final ModulesHolder m_MODULES;

    /**
     * A constructor that takes ModulesHolder
     *
     * @param modules The ModulesHolder
     */
    Swerve(ModulesHolder modules, SwerveConfigurator.KinematicsConfig kinematicsConfig)
    {
        m_MODULES = modules;
        m_kinematicsConfig = kinematicsConfig;
    }

    /**
     * A function to get the Swerve instance
     *
     * @return Swerve subsystem
     */
    public static Swerve getInstance() {
        return m_instance;
    }

    /**
     * A function to get a drive Command
     *
     * @param velocityMPS    wanted linear robot velocity
     * @param omegaRadPerSec wanted rotational robot velocity
     * @return driveCommand
     */
    public Command driveCommand(
            Supplier<Vector2D> velocityMPS,
            DoubleSupplier omegaRadPerSec,
            BooleanSupplier fieldOriented) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    Vector2D velocitySetPoint = getSmartTranslationalVelocitySetpoint(velocityMPS.get());
                    if (fieldOriented.getAsBoolean()) {
                        velocitySetPoint = velocitySetPoint.rotate(getYaw());
                    }
                    m_MODULES.setVelocities(velocitySetPoint, omegaRadPerSec.getAsDouble());
                },
                interrupt -> {
                    if (!interrupt) m_MODULES.stop();
                },
                () -> false
        );
    }

    /**
     * A function to get the velocity of the robot
     *
     * @return robot's velocity
     */
    public Vector2D getVelocity() {
        return m_MODULES.getVelocity();
    }

    public Rotation2d getYaw() {
        return null;
    }

    /**
     * A function to get the translational velocity setpoint
     *
     * @param velocitySetpoint wanted velocity setpoint
     * @return translational velocity setpoint
     */
    private Vector2D getSmartTranslationalVelocitySetpoint(Vector2D velocitySetpoint) {
        Vector2D deltaVelocity = getVelocity().plus(
                velocitySetpoint.mul(-1));
        Vector2D actualDeltaVelocity = applyAccelerationLimits(deltaVelocity);
        return getVelocity().plus(actualDeltaVelocity);
    }

    /**
     * A function to apply the acceleration limits
     *
     * @param velocityError wanted velocity
     * @return velocity limited by acceleration limits
     */
    private Vector2D applyAccelerationLimits(Vector2D velocityError) {
        Vector2D wantedAcceleration = velocityError.mul(1 / CYCLE_TIME);

        wantedAcceleration = applyForwardLimit(wantedAcceleration);
        wantedAcceleration = applyTiltLimit(wantedAcceleration);

        wantedAcceleration = applySkidLimit(wantedAcceleration);
        return wantedAcceleration.mul(CYCLE_TIME);
    }

    /**
     * A function to apply the forward acceleration limit
     *
     * @param wantedAcceleration the wanted acceleration
     * @return wanted acceleration limited by forward acceleration limit
     */
    private Vector2D applyForwardLimit(Vector2D wantedAcceleration) {
        double maxAcceleration =
                m_kinematicsConfig.MAX_FORWARD_ACCELERATION_MPSS *
                        (1 - (getVelocity().getDistance() / m_kinematicsConfig.MAX_VELOCITY_MPS));

        Vector2D rotatedWantedAcceleration = wantedAcceleration.rotate(
                wantedAcceleration.getDirection().minus(getVelocity().getDirection()));
        rotatedWantedAcceleration = new Vector2D(
                minSize(rotatedWantedAcceleration.getX(), maxAcceleration),
                rotatedWantedAcceleration.getDirection());
        wantedAcceleration = rotatedWantedAcceleration.rotate(
                getVelocity().getDirection().minus(wantedAcceleration.getDirection()));

        return wantedAcceleration;
    }

    /**
     * A function to apply the tilt acceleration limit
     *
     * @param wantedAcceleration the wanted acceleration
     * @return wanted acceleration limited by tilt acceleration limit
     */
    private Vector2D applyTiltLimit(Vector2D wantedAcceleration) {
        double frontAcceleration = minSize(wantedAcceleration.getX(), m_kinematicsConfig.MAX_FRONT_ACCELERATION);
        double sideAcceleration = minSize(wantedAcceleration.getY() ,m_kinematicsConfig.MAX_SIDE_ACCELERATION);
        return new Vector2D(frontAcceleration, sideAcceleration);
    }

    /**
     * A function to apply the skid acceleration limit
     *
     * @param wantedAcceleration the wanted acceleration
     * @return wanted acceleration limited by skid acceleration limit
     */
    private Vector2D applySkidLimit(Vector2D wantedAcceleration) {
        return new Vector2D(
                minSize(wantedAcceleration.getDistance(), m_kinematicsConfig.MAX_SKID_ACCELERATION_MPSS),
                wantedAcceleration.getDirection());
    }

    /**
     * A function to apply the forward acceleration limit
     *
     * @param sizeLimit max size
     * @return minimum double value
     */
    private static double minSize(double val, double sizeLimit) {
        return Math.min(sizeLimit, Math.abs(val)) * Math.signum(val);
    }
}