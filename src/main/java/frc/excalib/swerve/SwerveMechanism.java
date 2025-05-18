package frc.excalib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.math.Vector2D;
import frc.excalib.slam.mapper.odometry.Odometry;
import frc.excalib.slam.mapper.odometry.VisionMeasurement;
import frc.excalib.swerve.swerve_utils.SwerveAccUtils;
import frc.excalib.swerve.swerve_utils.SwerveSpecs;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.math.geometry.Rotation2d.kPi;
import static frc.excalib.swerve.swerve_utils.SwerveAccUtils.getSmartTranslationalVelocitySetPoint;
import static monologue.Annotations.Log;

/**
 * A class representing a swerve subsystem.
 */
public class SwerveMechanism implements Logged {
    private final ModulesHolder m_MODULES;
    private final IMU m_imu;
    private final SwerveSpecs m_specs;
    private final Odometry m_odometry;
    private ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();

    private final SwerveDriveKinematics m_swerveDriveKinematics;

    /**
     * A constructor that initialize the Swerve Subsystem
     *
     * @param modules         The ModulesHolder containing all swerve modules.
     * @param imu             IMU sensor.
     * @param initialPosition The initial position of the robot.
     */
    public SwerveMechanism(ModulesHolder modules,
                           SwerveSpecs swerveSpecs,
                           IMU imu,
                           Pose2d initialPosition) {
        this.m_MODULES = modules;
        this.m_imu = imu;
        m_imu.resetIMU();

        this.m_odometry = new Odometry(
                modules.getSwerveDriveKinematics(),
                modules.getModulesPositions(),
                m_imu::getZRotation,
                initialPosition
        );

        m_specs = swerveSpecs;
        SwerveAccUtils.setSwerveSpecs(m_specs);

        m_swerveDriveKinematics = m_MODULES.getSwerveDriveKinematics();

        putSmartDashboardData();
    }

    /**
     * Creates a drive command for the swerve drive.
     *
     * @param velocityMPS    Supplier for the desired linear velocity in meters per second.
     * @param omegaRadPerSec Supplier for the desired rotational velocity in radians per second.
     * @param fieldOriented  Supplier indicating whether the control is field-oriented.
     * @return A command that drives the robot.
     */
    public Command driveCommand(
            Supplier<Vector2D> velocityMPS,
            DoubleSupplier omegaRadPerSec,
            BooleanSupplier fieldOriented,
            boolean withAccLimits,
            SubsystemBase... requirements) {

        Supplier<Vector2D> adjustedVelocitySupplier = () -> {
            Vector2D velocity = getSmartTranslationalVelocitySetPoint(getVelocity(), velocityMPS.get(), withAccLimits);
            if (fieldOriented.getAsBoolean()) {
                Rotation2d yaw = getRotation2D().unaryMinus();
                if (!AllianceUtils.isBlueAlliance()) yaw = yaw.plus(kPi);
                return velocity.rotate(yaw);
            }
            return velocity;
        };

        Command driveCommand = new ParallelCommandGroup(
                m_MODULES.setVelocitiesCommand(
                        adjustedVelocitySupplier,
                        omegaRadPerSec
                ),
                new RunCommand(
                        () -> m_desiredChassisSpeeds = new ChassisSpeeds(
                                adjustedVelocitySupplier.get().getX(),
                                adjustedVelocitySupplier.get().getY(),
                                omegaRadPerSec.getAsDouble()
                        )
                )
        ).withName("Drive Command");
        driveCommand.addRequirements(requirements);
        return driveCommand;
    }

    /**
     * A non-command function that drives the robot (for pathplanner).
     *
     * @param speeds A ChassisSpeeds object represents ROBOT RELATIVE speeds desired speeds.
     */
    public void driveRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = m_swerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_specs.maxVelocity());
        m_MODULES.setModulesStates(desiredStates);
    }

    /**
     * Creates a Command that stops the swerve
     *
     * @return A command that stops the swerve
     */
    public Command stopCommand() {
        return new InstantCommand(m_MODULES::stop);
    }

    public Command coastCommand() {
        return m_MODULES.coastCommand().ignoringDisable(true).withName("Coast Command");
    }

    /**
     * Updates the robot's odometry.
     */
    public void updateOdometry(VisionMeasurement... visionMeasurements) {
        m_odometry.updateOdometry(m_MODULES.getModulesPositions());

        for (VisionMeasurement measurement : visionMeasurements) {
            m_odometry.addVisionMeasurement(measurement.estimatedRobotPose(), measurement.timestampSeconds());
        }
    }

    public Command resetAngleCommand() {
        return new InstantCommand(m_imu::resetIMU).ignoringDisable(true);
    }

    /**
     * A method that restarts the odometry.
     *
     * @param newPose the wanted new Pose2d of the robot.
     */
    public void resetOdometry(Pose2d newPose) {
        m_odometry.resetOdometry(m_MODULES.getModulesPositions(), newPose);
    }

    /**
     * Gets the robot's rotation.
     *
     * @return The current rotation of the robot.
     */
    @Log.NT(key = "Robot Angle")
    public Rotation2d getRotation2D() {
        return getPose2D().getRotation();
    }

    /**
     * Gets the robot's pose.
     *
     * @return The current pose of the robot.
     */
    @Log.NT(key = "Robot Pose")
    public Pose2d getPose2D() {
        return m_odometry.getRobotPose();
    }

    /**
     * Gets the current velocity of the robot.
     *
     * @return The robot's velocity as a Vector2D.
     */
    @Log.NT(key = "Robot Velocity")
    public Vector2D getVelocity() {
        return m_MODULES.getVelocity();
    }

//    @Log.NT(key = "Robot Velocity Distance")
//    public double getVelocityDistance() {
//        return m_MODULES.getVelocityDistance();
//    }

    @Log.NT(key = "Robot Angular Velocity")
    public double getOmegaRadPerSec() {
        return m_MODULES.getOmegaRadPerSec();
    }

    /**
     * Gets the current acceleration of the robot.
     *
     * @return The robot's acceleration as a Vector2D.
     */
    @Log.NT(key = "Robot Acceleration")
    public double getAccelerationDistance() {
        return new Vector2D(m_imu.getAccX(), m_imu.getAccY()).getDistance();
    }

    /**
     * Gets the current robot relative speed.
     *
     * @return The robot's speed as a ChassisSpeeds.
     */
    @Log.NT(key = "Measured Chassis Speeds")
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return m_swerveDriveKinematics.toChassisSpeeds(m_MODULES.getStates());
    }

    @Log.NT(key = "Desired Chassis Speeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return m_desiredChassisSpeeds;
    }

    @Log.NT(key = "Modules States")
    public SwerveModuleState[] getModulesStates() {
        return m_MODULES.getStates();
    }

    @Log.NT(key = "Modules Desired States")
    public SwerveModuleState[] getDesiredModulesStates() {
        return m_MODULES.getDesiredStates();
    }

    /**
     * A function that initialize the Swerve tab for Elastic.
     */
    public void putSmartDashboardData() {
        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> m_MODULES.m_frontLeft.getPosition().getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> m_MODULES.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Front Right Angle", () -> m_MODULES.m_frontRight.getPosition().getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> m_MODULES.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Back Left Angle", () -> m_MODULES.m_backLeft.getPosition().getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> m_MODULES.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Back Right Angle", () -> m_MODULES.m_backRight.getPosition().getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> m_MODULES.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation2D().getRadians(), null);
        });
    }

    /**
     * Runs a system identification routine on a specific module's angle.
     *
     * @param module  The module index (0-3).
     * @param dir     The direction of the sysid routine.
     * @param dynamic Whether to perform a dynamic or quasistatic test.
     * @return The command to perform the sysid routine.
     */
    public Command driveSingleSysId(int module, Direction dir, SubsystemBase swerve, SysidConfig sysidConfig, boolean dynamic) {
        SwerveModule selectedModule;

        switch (module) {
            case 0 -> selectedModule = m_MODULES.m_frontLeft;
            case 1 -> selectedModule = m_MODULES.m_frontRight;
            case 2 -> selectedModule = m_MODULES.m_backLeft;
            case 3 -> selectedModule = m_MODULES.m_backRight;
            default -> throw new IllegalArgumentException("Invalid module index: " + module);
        }

        return dynamic ?
                selectedModule.driveSysIdDynamic(dir, swerve, sysidConfig)
                : selectedModule.driveSysIdQuas(dir, swerve, sysidConfig);
    }

    /**
     * Runs a system identification routine on a specific module's angle.
     *
     * @param module  The module index (0-3).
     * @param dir     The direction of the sysid routine.
     * @param dynamic Whether to perform a dynamic or quasistatic test.
     * @return The command to perform the sysid routine.
     */
    public Command steeringSingleSysId(int module, Direction dir, SubsystemBase swerve, SysidConfig sysidConfig, boolean dynamic) {
        SwerveModule selectedModule;

        switch (module) {
            case 0 -> selectedModule = m_MODULES.m_frontLeft;
            case 1 -> selectedModule = m_MODULES.m_frontRight;
            case 2 -> selectedModule = m_MODULES.m_backLeft;
            case 3 -> selectedModule = m_MODULES.m_backRight;
            default -> throw new IllegalArgumentException("Invalid module index: " + module);
        }

        return dynamic ?
                selectedModule.angleSysIdDynamic(dir, swerve, sysidConfig)
                : selectedModule.angleSysIdQuas(dir, swerve, sysidConfig);
    }

    public void periodic() {
        m_MODULES.periodic();
    }
}
