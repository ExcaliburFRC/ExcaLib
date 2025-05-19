package frc.excalib.swerve;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.excalib.control.math.Vector2D;
import monologue.Logged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ModulesHolder implements Logged {
    public final SwerveModule m_frontLeft;
    public final SwerveModule m_frontRight;
    public final SwerveModule m_backLeft;
    public final SwerveModule m_backRight;

    private final SwerveDriveKinematics m_swerveDriveKinematics;

    private final SwerveModulePosition[] m_modulePositions;

    /**
     * A constructor that initialize the ModulesHolder.
     *
     * @param frontLeft  A SwerveModule represents the front-left module.
     * @param frontRight A SwerveModule represents the front-right module.
     * @param backLeft   A SwerveModule represents the back-left module.
     * @param backRight  A SwerveModule represents the back-right module.
     */
    public ModulesHolder(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight) {
        this.m_frontLeft = frontLeft;
        this.m_frontRight = frontRight;
        this.m_backLeft = backLeft;
        this.m_backRight = backRight;

        this.m_swerveDriveKinematics = new SwerveDriveKinematics(
                frontLeft.m_MODULE_LOCATION,
                frontRight.m_MODULE_LOCATION,
                backLeft.m_MODULE_LOCATION,
                backRight.m_MODULE_LOCATION
        );

        m_modulePositions = new SwerveModulePosition[]{
                m_frontLeft.getModulePosition(),
                m_frontRight.getModulePosition(),
                m_backLeft.getModulePosition(),
                m_backRight.getModulePosition()
        };
    }

    /**
     * A method that calculates the minimum velocity ratio limit among all modules.
     *
     * @param translationVelocity The desired translation velocity of the robot.
     * @param omegaRadPerSec      The desired angular velocity of the robot in radians per second.
     * @return The velocity ratio limit.
     */
    private double calcVelocityRatioLimit(Vector2D translationVelocity, double omegaRadPerSec) {
        double velocityRatioLimit = Math.min(
                Math.min(
                        m_frontLeft.getVelocityRatioLimit(translationVelocity, omegaRadPerSec),
                        m_frontRight.getVelocityRatioLimit(translationVelocity, omegaRadPerSec)
                ),
                Math.min(
                        m_backLeft.getVelocityRatioLimit(translationVelocity, omegaRadPerSec),
                        m_backRight.getVelocityRatioLimit(translationVelocity, omegaRadPerSec)
                )
        );
        return Math.min(1.0, velocityRatioLimit); // Ensure the limit does not exceed 1.0
    }

    /**
     * A method that sets the velocities of all modules based on the desired translation and rotation velocities of the robot.
     *
     * @param translationalVel The desired translation velocity supplier.
     * @param omega            The desired angular velocity supplier.
     * @return A command that sets the velocities.
     */
    Command setVelocitiesCommand(Supplier<Vector2D> translationalVel, DoubleSupplier omega) {
        return new ParallelCommandGroup(
                m_frontLeft.setVelocityCommand(
                        translationalVel,
                        omega,
                        () -> calcVelocityRatioLimit(translationalVel.get(), omega.getAsDouble())
                ),
                m_frontRight.setVelocityCommand(
                        translationalVel,
                        omega,
                        () -> calcVelocityRatioLimit(translationalVel.get(), omega.getAsDouble())
                ),
                m_backLeft.setVelocityCommand(
                        translationalVel,
                        omega,
                        () -> calcVelocityRatioLimit(translationalVel.get(), omega.getAsDouble())
                ),
                m_backRight.setVelocityCommand(
                        translationalVel,
                        omega,
                        () -> calcVelocityRatioLimit(translationalVel.get(), omega.getAsDouble())
                )
        );
    }

    /**
     * A method that stops all swerve modules.
     */
    void stop() {
        m_frontLeft.stopModule();
        m_frontRight.stopModule();
        m_backLeft.stopModule();
        m_backRight.stopModule();
    }

    /**
     * @return A Command that sets the idle state of all modules to coast.
     */
    Command coastCommand() {
        return new ParallelCommandGroup(
                m_frontLeft.coastCommand(),
                m_frontRight.coastCommand(),
                m_backLeft.coastCommand(),
                m_backRight.coastCommand()
        );
    }

    /**
     * A method that sets the states of the modules to the desired states.
     *
     * @param states an array represents the wanted states of the modules.
     */
    void setModulesStates(SwerveModuleState[] states) {
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    /**
     * A method to get the robot's average sigma velocity based on the velocities of all modules.
     *
     * @return a Vector2D represents the robot's velocity.
     */
    Vector2D getSigmaVelocity() {
        // Sum the velocities of all modules
        double totalX = m_frontLeft.getVelocity().getX()
                + m_frontRight.getVelocity().getX()
                + m_backLeft.getVelocity().getX()
                + m_backRight.getVelocity().getX();

        double totalY = m_frontLeft.getVelocity().getY()
                + m_frontRight.getVelocity().getY()
                + m_backLeft.getVelocity().getY()
                + m_backRight.getVelocity().getY();

        // Compute the average velocity
        return new Vector2D(totalX * 0.25, totalY * 0.25);
    }

    /**
     * A method to get the linear velocity of the robot.
     *
     * @return the linear velocity of the robot.
     */
    double getLinearVelocity() {
        return new Vector2D(
                m_swerveDriveKinematics.toChassisSpeeds(getStates()).vxMetersPerSecond,
                m_swerveDriveKinematics.toChassisSpeeds(getStates()).vyMetersPerSecond
        ).getDistance();
    }

    /**
     * A method to get the angular velocity of the robot.
     *
     * @return the angular velocity of the robot.
     */
    double getOmegaRadPerSec() {
        return m_swerveDriveKinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond;
    }

    /**
     * A method to get the states of all modules.
     *
     * @return An array of SwerveModuleState representing the states of the modules.
     */
    SwerveModuleState[] getStates() {
        return new SwerveModuleState[]{
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState()
        };
    }

    /**
     * A method to get the desired states of all modules.
     *
     * @return An array of SwerveModuleState representing the desired states of the modules.
     */
    SwerveModuleState[] getDesiredStates() {
        return new SwerveModuleState[]{
                m_frontLeft.getDesiredState(),
                m_frontRight.getDesiredState(),
                m_backLeft.getDesiredState(),
                m_backRight.getDesiredState()
        };
    }

    /**
     * Gets the swerve drive kinematics.
     *
     * @return The SwerveDriveKinematics instance.
     */
    SwerveDriveKinematics getSwerveDriveKinematics() {
        return m_swerveDriveKinematics;
    }

    /**
     * Gets the positions of all modules.
     *
     * @return An array of SwerveModulePosition representing the positions of the modules.
     */
    SwerveModulePosition[] getModulesPositions() {
        return m_modulePositions;
    }

    /**
     * A method that updates the modules positions of all modules. should be called periodically.
     */
    public void periodic() {
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_backLeft.periodic();
        m_backRight.periodic();
    }
}
