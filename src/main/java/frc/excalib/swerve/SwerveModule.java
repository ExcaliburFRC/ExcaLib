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
 * A class representing a swerve module
 *
 * @author Yoav Cohen & Itay Keller
 */
public class SwerveModule implements Logged {
    private final FlyWheel m_driveWheel;
    private final Turret m_turret;
    public final Translation2d m_MODULE_LOCATION;
    private final double m_MAX_VEL;
    private final Rotation2d m_moduleAnglePlus90;
    private final Vector2D m_setPoint = new Vector2D(0, 0);
    private final SwerveModulePosition m_swerveModulePosition;

    /**
     * A constructor for the SwerveModule
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

        // Precompute the rotated module angle (module angle + 90 degrees)
        m_moduleAnglePlus90 = m_MODULE_LOCATION.getAngle().plus(new Rotation2d(Math.PI / 2));

        m_swerveModulePosition = new SwerveModulePosition(m_driveWheel.logPosition(), m_turret.getPosition());
    }

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

    Vector2D getSigmaVelocity(Vector2D translationVelocity, double omegaRadPerSec, double velocityRatioLimit) {
        Vector2D rotationVector = new Vector2D(
                omegaRadPerSec,
                m_moduleAnglePlus90
        );
        Vector2D sigmaVel = translationVelocity.plus(rotationVector);
        sigmaVel = sigmaVel.mul(velocityRatioLimit);
        return sigmaVel;
    }

    public boolean isOptimizable(Vector2D moduleVelocitySetPoint) {
        Rotation2d setPointDirection = moduleVelocitySetPoint.getDirection();
        Rotation2d currentDirection = m_turret.getPosition();
        double deltaDirection = Math.cos(setPointDirection.minus(currentDirection).getRadians());

        // If the dot product is negative, reversing the wheel direction may be beneficial
        return deltaDirection < 0;
    }

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
                        return m_turret.getPosition();
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

    public Command setVelocityCommand(
            Supplier<Vector2D> translationVelocity,
            DoubleSupplier omegaRadPerSec,
            DoubleSupplier velocityRatioLimit) {

        return setVelocityCommand(() -> getSigmaVelocity(
                translationVelocity.get(),
                omegaRadPerSec.getAsDouble(),
                velocityRatioLimit.getAsDouble()));
    }

    public Command coastCommand() {
        return new ParallelCommandGroup(
                m_driveWheel.coastCommand(),
                m_turret.coastCommand()
        );
    }

    public void setDesiredState(SwerveModuleState wantedState) {
        Vector2D velocity = new Vector2D(wantedState.speedMetersPerSecond, wantedState.angle);
        double speed = velocity.getDistance();
        Rotation2d direction = velocity.getDirection();

        if (speed < 0.1) {
            speed = 0.0;
            direction = m_turret.getPosition();
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
     * Gets the module's velocity.
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
        return m_turret.getPosition();
    }

    public SwerveModulePosition getModulePosition() {
        return m_swerveModulePosition;
    }

    public SwerveModuleState logState() {
        Vector2D velocity = getVelocity();
        return new SwerveModuleState(velocity.getDistance(), velocity.getDirection());
    }

    public SwerveModuleState logSetpointState() {
        return new SwerveModuleState(m_setPoint.getDistance(), m_setPoint.getDirection());
    }


    public Command driveSysIdDynamic(SysIdRoutine.Direction direction, Swerve swerve, SysidConfig sysidConfig) {
        return m_driveWheel.sysIdDynamic(direction, swerve, m_driveWheel::logPosition, sysidConfig, false);
    }

    public Command driveSysIdQuas(SysIdRoutine.Direction direction, Swerve swerve, SysidConfig sysidConfig) {
        return m_driveWheel.sysIdQuasistatic(direction, swerve, m_driveWheel::logPosition, sysidConfig, false);
    }

    public Command angleSysIdDynamic(SysIdRoutine.Direction direction, Swerve swerve, SysidConfig sysidConfig) {
        return m_turret.sysIdDynamic(direction, swerve, m_turret::logPosition, sysidConfig, false);
    }

    public Command angleSysIdQuas(SysIdRoutine.Direction direction, Swerve swerve, SysidConfig sysidConfig) {
        return m_turret.sysIdQuasistatic(direction, swerve, m_turret::logPosition, sysidConfig, false);
    }

    public void periodic() {
        m_swerveModulePosition.distanceMeters = m_driveWheel.logPosition();
        m_swerveModulePosition.angle = m_turret.getPosition();
    }
}
