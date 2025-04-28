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

import static frc.robot.SwerveConstants.MAX_VEL;
import static monologue.Annotations.Log;

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

        // Initialize SwerveDriveKinematics once since module locations are constant
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
     * Stops all swerve modules.
     */
    public void stop() {
        m_frontLeft.stopModule();
        m_frontRight.stopModule();
        m_backLeft.stopModule();
        m_backRight.stopModule();
    }

    /**
     * Gets the robot's average velocity based on the velocities of all modules.
     *
     * @return a Vector2D representing the robot's velocity.
     */
    public Vector2D getVelocity() {
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

    @Log.NT(key = "angular vel")
    public double getOmegaRadPerSec() {
        return new SwerveDriveKinematics(
                m_frontLeft.m_MODULE_LOCATION,
                m_frontRight.m_MODULE_LOCATION,
                m_backLeft.m_MODULE_LOCATION,
                m_backRight.m_MODULE_LOCATION
        ).toChassisSpeeds(logStates()).omegaRadiansPerSecond;
    }

    @Log.NT(key = "swerve velocity")
    public double getVelocityDistance() {
        return getVelocity().getDistance();
    }

    /**
     * Calculates the minimum velocity ratio limit among all modules.
     *
     * @param translationVelocity The desired translation velocity.
     * @param omegaRadPerSec      The desired rotation rate in radians per second.
     * @return The velocity ratio limit.
     */
    private double calcVelocityRatioLimit(Vector2D translationVelocity, double omegaRadPerSec) {
        double flLimit = m_frontLeft.getVelocityRatioLimit(translationVelocity, omegaRadPerSec);
        double frLimit = m_frontRight.getVelocityRatioLimit(translationVelocity, omegaRadPerSec);
        double blLimit = m_backLeft.getVelocityRatioLimit(translationVelocity, omegaRadPerSec);
        double brLimit = m_backRight.getVelocityRatioLimit(translationVelocity, omegaRadPerSec);

        double velocityRatioLimit = Math.min(Math.min(flLimit, frLimit), Math.min(blLimit, brLimit));
        return Math.min(1.0, velocityRatioLimit); // Ensure the limit does not exceed 1.0
    }

    /**
     * Sets the velocities of all modules based on the desired translation and rotation velocities.
     *
     * @param omega            The desired rotation rate supplier.
     * @param translationalVel The desired translation velocity supplier.
     * @return A command to set the velocities.
     */
    public Command setVelocitiesCommand(Supplier<Vector2D> translationalVel, DoubleSupplier omega) {
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

    public Command coastCommand() {
        return new ParallelCommandGroup(
                m_frontLeft.coastCommand(),
                m_frontRight.coastCommand(),
                m_backLeft.coastCommand(),
                m_backRight.coastCommand()
        );
    }

    public void setModulesStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VEL);
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    /**
     * Logs the states of all modules.
     *
     * @return An array of SwerveModuleState representing the states of the modules.
     */
    @Log.NT(key = "Swerve States")
    public SwerveModuleState[] logStates() {
        return new SwerveModuleState[]{
                m_frontLeft.logState(),
                m_frontRight.logState(),
                m_backLeft.logState(),
                m_backRight.logState()
        };
    }

    @Log.NT(key = "SetPoints")
    public SwerveModuleState[] logSetPointStates() {
        return new SwerveModuleState[]{
                m_frontLeft.logSetpointState(),
                m_frontRight.logSetpointState(),
                m_backLeft.logSetpointState(),
                m_backRight.logSetpointState()
        };
    }

    /**
     * Gets the swerve drive kinematics.
     *
     * @return The SwerveDriveKinematics instance.
     */
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return m_swerveDriveKinematics;
    }

    /**
     * Gets the positions of all modules.
     *
     * @return An array of SwerveModulePosition representing the positions of the modules.
     */
    public SwerveModulePosition[] getModulesPositions() {
        return m_modulePositions;
    }

    public void periodic() {
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_backLeft.periodic();
        m_backRight.periodic();
    }
}
