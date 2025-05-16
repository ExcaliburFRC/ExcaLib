package frc.excalib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.math.Vector2D;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.excalib.mechanisms.turret.Turret;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.math.geometry.Rotation2d.kPi;

/**
 * A class representing a swerve module.
 *
 * @author Yoav Cohen & Itay Keller.
 */
public class SwerveModule implements Logged {
    private final FlyWheel m_drivingMechanism;
    private final Turret m_steeringMechanism;

    private final SwerveModulePosition m_swerveModulePosition;

    public final Translation2d m_MODULE_LOCATION;
    private final Rotation2d m_moduleAnglePlus90;
    private final double kMaxVel;

    private final Vector2D m_setPoint = new Vector2D(0, 0);

    /**
     * A constructor for the SwerveModule.
     *
     * @param drivingMechanism  the drive wheel presented as FlyWheel.
     * @param steeringMechanism the steering mechanism presented as Turret.
     * @param moduleLocation    the location of the module relative to the center of the robot.
     * @param maxVel            the max velocity of the module.
     */
    public SwerveModule(FlyWheel drivingMechanism, Turret steeringMechanism,
                        Translation2d moduleLocation, double maxVel) {
        m_drivingMechanism = drivingMechanism;

        m_steeringMechanism = steeringMechanism;

        m_MODULE_LOCATION = moduleLocation;
        kMaxVel = maxVel;

        m_moduleAnglePlus90 = m_MODULE_LOCATION.getAngle().plus(new Rotation2d(Math.PI / 2));

        m_swerveModulePosition = new SwerveModulePosition(m_drivingMechanism.logPosition(), m_steeringMechanism.getPosition());
    }

    /**
     * A method to get the ratio limit of the module.
     *
     * @param translationVelocity the wanted linear velocity for the robot.
     * @param omegaRadPerSec      the wanted angular velocity for the robot.
     * @return the ratio limit of this module.
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
        return kMaxVel / sigmaVelDistance;
    }

    /**
     * A method to get the sigma velocity for the module.
     *
     * @param translationVelocity the wanted linear velocity for the robot.
     * @param omegaRadPerSec      the wanted angular velocity for the robot.
     * @param velocityRatioLimit  the velocity ratio limit for this module.
     * @return a Vector2D represents the sigma velocity.
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
     * A method to check if the module should be optimized.
     *
     * @param moduleVelocitySetPoint the wanted velocity setpoint for this module.
     * @return a boolean says if the module should be optimized.
     */
    boolean isOptimizable(Vector2D moduleVelocitySetPoint) {
        Rotation2d setPointDirection = moduleVelocitySetPoint.getDirection();
        Rotation2d currentDirection = m_steeringMechanism.getPosition();
        double deltaDirection = Math.cos(setPointDirection.minus(currentDirection).getRadians());

        // If the dot product is negative, reversing the wheel direction may be beneficial
        return deltaDirection < 0;
    }


    /**
     * A method that sets the velocity of the module to the wanted velocity.
     *
     * @param moduleVelocity a supplier of Vector2D represents the wanted velocity.
     * @return a Command that sets the module velocity to the wanted velocity.
     */
    public Command setVelocityCommand(Supplier<Vector2D> moduleVelocity) {
        return new ParallelCommandGroup(
                m_drivingMechanism.setDynamicVelocityCommand(() -> {
                    Vector2D velocity = moduleVelocity.get();
                    double speed = velocity.getDistance();

                    if (speed < 0.1) {
                        speed = 0;
                    }

                    boolean optimize = isOptimizable(velocity);
                    return optimize ? -speed : speed;
                }),
                m_steeringMechanism.setPositionCommand(() -> {
                    Vector2D velocity = moduleVelocity.get();
                    double speed = velocity.getDistance();

                    if (speed < 0.1) {
                        return m_steeringMechanism.getPosition();
                    }

                    boolean optimize = isOptimizable(velocity);
                    Rotation2d direction = velocity.getDirection();
                    return optimize ? direction.rotateBy(kPi) : direction;
                }),
                new RunCommand(() -> {
                    m_setPoint.setY(moduleVelocity.get().getY());
                    m_setPoint.setX(moduleVelocity.get().getX());
                })
        );
    }

    /**
     * A method that sets the velocity of the module using the overall robot wanted velocity.
     *
     * @param translationVelocity the wanted robot linear velocity.
     * @param omegaRadPerSec the wanted robot angular velocity.
     * @param velocityRatioLimit the needed ratio limit.
     * @return a Command that sets the module velocity to the wanted velocity.
     */
    public Command setVelocityCommand(
            Supplier<Vector2D> translationVelocity,
            DoubleSupplier omegaRadPerSec,
            DoubleSupplier velocityRatioLimit) {

        return setVelocityCommand(
                () -> getSigmaVelocity(
                        translationVelocity.get(),
                        omegaRadPerSec.getAsDouble(),
                        velocityRatioLimit.getAsDouble()
                )
        );
    }

    /**
     * A method that sets the state of the module to the desired state.
     *
     * @param wantedState the wanted module state.
     */
    public void setDesiredState(SwerveModuleState wantedState) {
        Vector2D velocity = new Vector2D(wantedState.speedMetersPerSecond, wantedState.angle);
        double speed = velocity.getDistance();
        Rotation2d direction = velocity.getDirection();

        if (speed < 0.1) {
            speed = 0.0;
            direction = m_steeringMechanism.getPosition();
        }

        boolean optimize = isOptimizable(velocity);

        m_drivingMechanism.setDynamicVelocity(optimize ? -speed : speed);
        m_steeringMechanism.setPosition(optimize ? direction.rotateBy(kPi) : direction);
    }

    /**
     * A method that stops the module by setting the drive wheel output to zero.
     */
    void stopModule() {
        m_drivingMechanism.setOutput(0);
    }

    /**
     * @return A Command that sets the idle state of the module's motors to coast.
     */
    public Command coastCommand() {
        return new ParallelCommandGroup(
                m_drivingMechanism.coastCommand(),
                m_steeringMechanism.coastCommand()
        );
    }

    /**
     * A method to get the module's velocity.
     *
     * @return a Vector2D represents the module velocity.
     */
    Vector2D getVelocity() {
        return new Vector2D(m_drivingMechanism.getVelocity(), getPosition());
    }

    /**
     * A method to get the angle of the module.
     *
     * @return the angle of the module as Rotation2d.
     */
    Rotation2d getPosition() {
        return m_steeringMechanism.getPosition();
    }

    /**
     * A method to get the position of the module.
     *
     * @return the module position.
     */
    public SwerveModulePosition getModulePosition() {
        return m_swerveModulePosition;
    }

    /**
     * A method to get the current state of the module.
     *
     * @return the current state of the module.
     */
    public SwerveModuleState getState() {
        Vector2D velocity = getVelocity();
        return new SwerveModuleState(velocity.getDistance(), velocity.getDirection());
    }

    /**
     * A method to get the wanted state of the module.
     *
     * @return the wanted state of the module.
     */
    public SwerveModuleState getSetpointState() {
        return new SwerveModuleState(m_setPoint.getDistance(), m_setPoint.getDirection());
    }


    public Command driveSysIdDynamic(SysIdRoutine.Direction direction, SubsystemBase swerve, SysidConfig sysidConfig) {
        return m_drivingMechanism.sysIdDynamic(direction, swerve, m_drivingMechanism::logPosition, sysidConfig, false);
    }

    public Command driveSysIdQuas(SysIdRoutine.Direction direction, SubsystemBase swerve, SysidConfig sysidConfig) {
        return m_drivingMechanism.sysIdQuasistatic(direction, swerve, m_drivingMechanism::logPosition, sysidConfig, false);
    }

    public Command angleSysIdDynamic(SysIdRoutine.Direction direction, SubsystemBase swerve, SysidConfig sysidConfig) {
        return m_steeringMechanism.sysIdDynamic(direction, swerve, m_steeringMechanism::logPosition, sysidConfig, false);
    }

    public Command angleSysIdQuas(SysIdRoutine.Direction direction, SubsystemBase swerve, SysidConfig sysidConfig) {
        return m_steeringMechanism.sysIdQuasistatic(direction, swerve, m_steeringMechanism::logPosition, sysidConfig, false);
    }

    /**
     * A method that updates the module position. should be called periodically.
     */
    public void periodic() {
        m_drivingMechanism.periodic();

        m_swerveModulePosition.distanceMeters = m_drivingMechanism.logPosition();
        m_swerveModulePosition.angle = m_steeringMechanism.getPosition();
    }
}
