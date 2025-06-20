package frc.excalib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.math.Vector2D;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.excalib.mechanisms.turret.Turret;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;
import static frc.excalib.control.motor.motor_specs.DirectionState.REVERSE;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;

/**
 * A class representing a swerve module, which consists of a drive wheel and a turret for rotation.
 * Handles velocity and angle control, optimization, and system identification routines.
 */
public class SwerveModule implements Logged {
    /** The drive wheel mechanism for this swerve module. */
    private final FlyWheel m_driveWheel;
    /** The turret mechanism for module rotation. */
    private final Turret m_turret;
    /** The physical location of the module on the robot. */
    public final Translation2d m_MODULE_LOCATION;
    /** The maximum velocity for this module. */
    private final double m_MAX_VEL;
    /** The module's angle plus 90 degrees, used for rotation vector calculations. */
    private final Rotation2d m_moduleAnglePlus90;
    /** The current setpoint velocity vector for logging. */
    private final Vector2D m_setPoint = new Vector2D(0, 0);
    /** The current position of the swerve module (distance and angle). */
    private final SwerveModulePosition m_swerveModulePosition;

    /**
     * Constructs a SwerveModule with the given hardware and configuration.
     *
     * @param driveMotor The motor controlling the drive wheel.
     * @param rotationMotor The motor controlling the turret rotation.
     * @param angleGains PID gains for angle control.
     * @param velocityGains PID gains for velocity control.
     * @param PIDTolerance Tolerance for angle PID.
     * @param moduleLocation The location of this module on the robot.
     * @param angleSupplier Supplier for the current angle.
     * @param maxVel The maximum velocity for this module.
     * @param velocityConversionFactor Conversion factor for velocity units.
     * @param positionConversionFactor Conversion factor for position units.
     * @param rotationVelocityConversionFactor Conversion factor for rotation velocity.
     */
    public SwerveModule(Motor driveMotor, Motor rotationMotor, Gains angleGains, Gains velocityGains,
                        double PIDTolerance, Translation2d moduleLocation, DoubleSupplier angleSupplier,
                        double maxVel, double velocityConversionFactor, double positionConversionFactor,
                        double rotationVelocityConversionFactor) {
        driveMotor.setInverted(FORWARD);
        driveMotor.setVelocityConversionFactor(velocityConversionFactor);
        driveMotor.setIdleState(BRAKE);
        driveMotor.setPositionConversionFactor(positionConversionFactor);
        driveMotor.setCurrentLimit(0, 60);

        rotationMotor.setIdleState(BRAKE);
        rotationMotor.setVelocityConversionFactor(rotationVelocityConversionFactor);
        rotationMotor.setInverted(REVERSE);

        m_driveWheel = new FlyWheel(driveMotor, 10, 10, velocityGains);

        m_turret = new Turret(rotationMotor, new ContinuousSoftLimit(() -> Double.NEGATIVE_INFINITY, () -> Double.POSITIVE_INFINITY),
                angleGains, PIDTolerance, angleSupplier);

        m_MODULE_LOCATION = moduleLocation;
        m_MAX_VEL = maxVel;

        m_moduleAnglePlus90 = m_MODULE_LOCATION.getAngle().plus(new Rotation2d(Math.PI / 2));

        m_swerveModulePosition = new SwerveModulePosition(m_driveWheel.getPosition(), m_turret.getTurretPosition());
    }

    /**
     * Calculates the velocity ratio limit for the module based on translation and rotation velocities.
     *
     * @param translationVelocity The desired translation velocity vector.
     * @param omegaRadPerSec The desired rotational velocity (radians per second).
     * @return The ratio limit to ensure the module does not exceed its max velocity.
     */
    double getVelocityRatioLimit(Vector2D translationVelocity, double omegaRadPerSec) {
        Vector2D rotationVector = new Vector2D(
                omegaRadPerSec,
                m_moduleAnglePlus90
        );
        Vector2D sigmaVel = translationVelocity.plus(rotationVector);
        double sigmaVelDistance = sigmaVel.getDistance();

        // Avoid division by zero
        if (sigmaVelDistance == 0) {
            return 0;
        }
        return m_MAX_VEL / sigmaVelDistance;
    }

    /**
     * Computes the combined velocity vector (translation + rotation), scaled by a limit.
     *
     * @param translationVelocity The translation velocity vector.
     * @param omegaRadPerSec The rotational velocity (radians per second).
     * @param velocityRatioLimit The scaling limit for the velocity.
     * @return The resulting velocity vector.
     */
    Vector2D getSigmaVelocity(Vector2D translationVelocity, double omegaRadPerSec, double velocityRatioLimit) {
        Vector2D rotationVector = new Vector2D(
                omegaRadPerSec,
                m_moduleAnglePlus90
        );
        Vector2D sigmaVel = translationVelocity.plus(rotationVector);
        sigmaVel = sigmaVel.mul(velocityRatioLimit);
        return sigmaVel;
    }

    /**
     * Determines if the module's velocity setpoint can be optimized by reversing direction.
     *
     * @param moduleVelocitySetPoint The desired velocity vector.
     * @return True if optimization (reversing) is beneficial, false otherwise.
     */
    public boolean isOptimizable(Vector2D moduleVelocitySetPoint) {
        Rotation2d setPointDirection = moduleVelocitySetPoint.getDirection();
        Rotation2d currentDirection = m_turret.getTurretPosition();
        double deltaDirection = Math.cos(setPointDirection.minus(currentDirection).getRadians());

        // If the dot product is negative, reversing the wheel direction may be beneficial
        return deltaDirection < 0;
    }

    /**
     * Creates a command to set the module's velocity using the given velocity supplier.
     *
     * @param moduleVelocity Supplier for the desired velocity vector.
     * @return A command that sets the drive and turret to the desired velocity and direction.
     */
    public Command setVelocityCommand(Supplier<Vector2D> moduleVelocity) {
        return new ParallelCommandGroup(
                m_driveWheel.setDynamicVelocityCommand(() -> {
                    Vector2D velocity = moduleVelocity.get();
                    double speed = velocity.getDistance();

                    if (speed < 0.1) {
                        speed = 0;
                    }

                    boolean optimize = isOptimizable(velocity);
                    return optimize ? -speed : speed;
                }),
                m_turret.setPositionCommand(() -> {
                    Vector2D velocity = moduleVelocity.get();
                    double speed = velocity.getDistance();

                    if (speed < 0.1) {
                        return m_turret.getTurretPosition();
                    }

                    boolean optimize = isOptimizable(velocity);
                    Rotation2d direction = velocity.getDirection();
                    return optimize ? direction.rotateBy(Rotation2d.fromRadians(Math.PI)) : direction;
                }),
                new RunCommand(() -> {
                    m_setPoint.setY(moduleVelocity.get().getY());
                    m_setPoint.setX(moduleVelocity.get().getX());
                })
        );
    }

    /**
     * Creates a command to set the module's velocity using translation, rotation, and a velocity limit.
     *
     * @param translationVelocity Supplier for the translation velocity vector.
     * @param omegaRadPerSec Supplier for the rotational velocity (radians per second).
     * @param velocityRatioLimit Supplier for the velocity ratio limit.
     * @return A command that sets the module's velocity accordingly.
     */
    public Command setVelocityCommand(
            Supplier<Vector2D> translationVelocity,
            DoubleSupplier omegaRadPerSec,
            DoubleSupplier velocityRatioLimit) {

        return setVelocityCommand(() -> getSigmaVelocity(
                translationVelocity.get(),
                omegaRadPerSec.getAsDouble(),
                velocityRatioLimit.getAsDouble()));
    }

    /**
     * Creates a command to set both the drive wheel and turret to coast mode.
     *
     * @return A command that coasts both mechanisms.
     */
    public Command coastCommand() {
        return new ParallelCommandGroup(
                m_driveWheel.coastCommand(),
                m_turret.coastCommand()
        );
    }

    /**
     * Sets the desired state (speed and angle) for the swerve module.
     *
     * @param wantedState The desired swerve module state.
     */
    public void setDesiredState(SwerveModuleState wantedState) {
        Vector2D velocity = new Vector2D(wantedState.speedMetersPerSecond, wantedState.angle);
        double speed = velocity.getDistance();
        Rotation2d direction = velocity.getDirection();

        if (speed < 0.1) {
            speed = 0.0;
            direction = m_turret.getTurretPosition();
        }

        boolean optimize = isOptimizable(velocity);

        m_driveWheel.setDynamicVelocity(optimize ? -speed : speed);
        m_turret.setPosition(optimize ? direction.rotateBy(Rotation2d.fromRadians(Math.PI)) : direction);
    }

    /**
     * Stops the module by setting the drive wheel output to zero.
     */
    void stopModule() {
        m_driveWheel.setOutput(0);
    }

    /**
     * Gets the module's velocity as a vector.
     *
     * @return a Vector2D representing the velocity.
     */
    Vector2D getVelocity() {
        return new Vector2D(m_driveWheel.getVelocity(), getPosition());
    }

    /**
     * Gets the turret's position.
     *
     * @return the current position of the turret.
     */
    Rotation2d getPosition() {
        return m_turret.getTurretPosition();
    }

    /**
     * Gets the current position of the swerve module (distance and angle).
     *
     * @return The SwerveModulePosition object.
     */
    public SwerveModulePosition getModulePosition() {
        return m_swerveModulePosition;
    }

    /**
     * Logs the current state (velocity and direction) of the module.
     *
     * @return The current SwerveModuleState.
     */
    public SwerveModuleState logState() {
        Vector2D velocity = getVelocity();
        return new SwerveModuleState(velocity.getDistance(), velocity.getDirection());
    }

    /**
     * Logs the current setpoint state (velocity and direction) of the module.
     *
     * @return The setpoint SwerveModuleState.
     */
    public SwerveModuleState logSetpointState() {
        return new SwerveModuleState(m_setPoint.getDistance(), m_setPoint.getDirection());
    }

    /**
     * Creates a SysId dynamic test command for the drive wheel.
     *
     * @param direction The direction for the test.
     * @param swerve The swerve drive system.
     * @param sysidConfig The SysId configuration.
     * @return The command for dynamic SysId testing.
     */
    public Command driveSysIdDynamic(SysIdRoutine.Direction direction, Swerve swerve, SysidConfig sysidConfig) {
        return m_driveWheel.sysIdDynamic(direction, swerve, m_driveWheel::getPosition, sysidConfig, false);
    }

    /**
     * Creates a SysId quasistatic test command for the drive wheel.
     *
     * @param direction The direction for the test.
     * @param swerve The swerve drive system.
     * @param sysidConfig The SysId configuration.
     * @return The command for quasistatic SysId testing.
     */
    public Command driveSysIdQuas(SysIdRoutine.Direction direction, Swerve swerve, SysidConfig sysidConfig) {
        return m_driveWheel.sysIdQuasistatic(direction, swerve, m_driveWheel::getPosition, sysidConfig, false);
    }

    /**
     * Creates a SysId dynamic test command for the turret angle.
     *
     * @param direction The direction for the test.
     * @param swerve The swerve drive system.
     * @param sysidConfig The SysId configuration.
     * @return The command for dynamic SysId testing.
     */
    public Command angleSysIdDynamic(SysIdRoutine.Direction direction, Swerve swerve, SysidConfig sysidConfig) {
        return m_turret.sysIdDynamic(direction, swerve, m_turret::getPosition, sysidConfig, false);
    }

    /**
     * Creates a SysId quasistatic test command for the turret angle.
     *
     * @param direction The direction for the test.
     * @param swerve The swerve drive system.
     * @param sysidConfig The SysId configuration.
     * @return The command for quasistatic SysId testing.
     */
    public Command angleSysIdQuas(SysIdRoutine.Direction direction, Swerve swerve, SysidConfig sysidConfig) {
        return m_turret.sysIdQuasistatic(direction, swerve, m_turret::getPosition, sysidConfig, false);
    }

    /**
     * Periodic update for the swerve module, updates the module position for logging and odometry.
     */
    public void periodic() {
        m_swerveModulePosition.distanceMeters = m_driveWheel.getPosition();
        m_swerveModulePosition.angle = m_turret.getTurretPosition();
    }
}