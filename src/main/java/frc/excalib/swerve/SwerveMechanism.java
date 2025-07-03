package frc.excalib.swerve;

import edu.wpi.first.epilogue.Logged;
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
import frc.excalib.control.odometry.Odometry;
import frc.excalib.control.odometry.VisionMeasurement;
import frc.excalib.swerve.swerve_utils.SwerveAccUtils;
import frc.excalib.swerve.swerve_utils.SwerveModuleOption;
import frc.excalib.swerve.swerve_utils.SwerveSpecs;
import frc.excalib.swerve.swerve_utils.SysIdRoutineOption;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.math.geometry.Rotation2d.kPi;
import static frc.excalib.swerve.swerve_utils.SwerveAccUtils.getSmartTranslationalVelocitySetpoint;

/**
 * A class representing a swerve subsystem.
 */
@Logged
public class SwerveMechanism {
    private final ModulesHolder m_modules;
    private final IMU m_imu;
    private final SwerveSpecs m_specs;
    private final Odometry m_odometry;
    private ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();

    /**
     * A constructor that initialize the Swerve Subsystem
     *
     * @param modules         The ModulesHolder containing all swerve modules.
     * @param imu             The IMU sensor.
     * @param initialPosition The initial position of the robot.
     */
    public SwerveMechanism(ModulesHolder modules, SwerveSpecs swerveSpecs, IMU imu, Pose2d initialPosition) {
        m_modules = modules;
        m_specs = swerveSpecs;
        m_imu = imu;

        m_odometry = new Odometry(
                modules.getSwerveDriveKinematics(),
                modules.getModulesPositions(),
                m_imu::getZRotation,
                initialPosition
        );

        SwerveAccUtils.setSwerveSpecs(m_specs);
        initModulesDashboard();
    }

    /**
     * Creates a drive command for the swerve drive.
     *
     * @param velocityMPS    Supplier for the desired linear velocity in meters per second.
     * @param omegaRadPerSec Supplier for the desired angular velocity in radians per second.
     * @param fieldOriented  Supplier indicating whether the control is field-oriented.
     * @param withAccLimits  whether to use all limits or only the skid limit
     * @return A command that drives the robot.
     */
    public Command driveCommand(Supplier<Vector2D> velocityMPS, DoubleSupplier omegaRadPerSec,
                                BooleanSupplier fieldOriented, boolean withAccLimits, SubsystemBase... requirements) {

        Supplier<Vector2D> adjustedVelocitySupplier = () -> adjustVectorByOrientation(
                getSmartTranslationalVelocitySetpoint(getSigmaRobotVelocity(), velocityMPS.get(), withAccLimits),
                fieldOriented
        );

        Command driveCommand = new ParallelCommandGroup(
                m_modules.setVelocitiesCommand(adjustedVelocitySupplier, omegaRadPerSec),
                new RunCommand(
                        () -> m_desiredChassisSpeeds = new ChassisSpeeds(
                                adjustedVelocitySupplier.get().getX(),
                                adjustedVelocitySupplier.get().getY(),
                                omegaRadPerSec.getAsDouble()
                        )
                )
        );

        driveCommand.setName("swerve drive command");
        driveCommand.addRequirements(requirements);
        return driveCommand;
    }

    /**
     * A non-command function that drives the robot (for pathplanner).
     *
     * @param speeds A ChassisSpeeds object represents ROBOT RELATIVE desired speeds.
     */
    public void driveRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = m_modules.getSwerveDriveKinematics().toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_specs.maxVelocity());
        m_modules.setModulesStates(desiredStates);
    }

    /**
     * Creates a Command that stops the swerve
     *
     * @return A command that stops the swerve
     */
    public Command stopCommand() {
        return new InstantCommand(m_modules::stop);
    }

    /**
     * @return A Command that sets the idle state of the swerve to coast.
     */
    public Command coastCommand() {
        return m_modules.coastCommand().ignoringDisable(true).withName("Coast Command");
    }

    /**
     * @return A command that resets the IMU angle.
     */
    public Command resetAngleCommand() {
        return new InstantCommand(m_imu::resetIMU).ignoringDisable(true);
    }

    /**
     * Updates the robot's odometry.
     *
     * @param visionMeasurements Optionally, you can add a vision measurement to the odometry.
     */
    public void updateOdometry(VisionMeasurement... visionMeasurements) {
        m_odometry.updateOdometry(m_modules.getModulesPositions());

        for (VisionMeasurement measurement : visionMeasurements) {
            m_odometry.addVisionMeasurement(measurement.estimatedRobotPose(), measurement.timestampSeconds());
        }
    }

    /**
     * A method that resets the odometry.
     *
     * @param newPose the wanted new Pose2d of the robot.
     */
    public void resetOdometry(Pose2d newPose) {
        m_odometry.resetOdometry(m_modules.getModulesPositions(), newPose);
    }

    /**
     * A method that adjusts the vector if driving field oriented
     *
     * @param velocity      current velocity vector
     * @param fieldOriented is driving field oriented
     * @return the fixed vector
     */
    private Vector2D adjustVectorByOrientation(Vector2D velocity, BooleanSupplier fieldOriented) {
        Rotation2d yaw;
        if (fieldOriented.getAsBoolean()) {
            yaw = getRotation2D().unaryMinus();
            if (AllianceUtils.isRedAlliance()) yaw = yaw.plus(kPi);
            return velocity.rotate(yaw);
        }
        return velocity;
    }

    /**
     * Gets the robot's heading.
     *
     * @return The current heading of the robot.
     */
    @Logged(name = "robot yaw angle")
    public Rotation2d getRotation2D() {
        return getPose2D().getRotation();
    }

    /**
     * Gets the robot's pose.
     *
     * @return The current pose of the robot.
     */
    @Logged(name = "robot pose")
    public Pose2d getPose2D() {
        return m_odometry.getRobotPose();
    }

    /**
     * Gets the current sigma velocity of the robot.
     *
     * @return The robot's sigma velocity as a Vector2D.
     */
    @Logged(name = "robot sigma velocity")
    public Vector2D getSigmaRobotVelocity() {
        return m_modules.getSigmaVelocity();
    }

    /**
     * Gets the current linear velocity of the robot.
     *
     * @return The robot's linear velocity.
     */
    @Logged(name = "robot linear velocity")
    public Vector2D getLinearRobotVelocity() {
        return m_modules.getLinearVelocity();
    }

    /**
     * Gets the current angular velocity of the robot.
     *
     * @return The robot's angular velocity.
     */
    @Logged(name = "robot angular velocity")
    public double getOmegaRadPerSec() {
        return m_modules.getOmegaRadPerSec();
    }

    /**
     * Gets the current acceleration of the robot.
     *
     * @return The robot's acceleration as a Vector2D.
     */
    @Logged(name = "robot acceleration")
    public Vector2D getRobotAcceleration() {
        return new Vector2D(m_imu.getAccX(), m_imu.getAccY());
    }

    /**
     * Gets the current robot relative speed.
     *
     * @return The robot's speed as a ChassisSpeeds.
     */
    @Logged(name = "measured chassis speeds")
    public ChassisSpeeds getRobotChassisSpeeds() {
        return m_modules.getSwerveDriveKinematics().toChassisSpeeds(m_modules.getStates());
    }

    /**
     * Gets the desired robot relative speed.
     *
     * @return The robot's desired speed as a ChassisSpeeds.
     */
    @Logged(name = "desired chassis speeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return m_desiredChassisSpeeds;
    }

    /**
     * Gets the current modules states.
     *
     * @return The robot's modules states.
     */
    @Logged(name = "modules states")
    public SwerveModuleState[] getModulesStates() {
        return m_modules.getStates();
    }

    /**
     * Gets the desired modules states.
     *
     * @return The robot's desired modules states.
     */
    @Logged(name = "modules desired states")
    public SwerveModuleState[] getDesiredModulesStates() {
        return m_modules.getDesiredStates();
    }

    /**
     * A function that initialize the Swerve data for the dashboard.
     */
    public void initModulesDashboard() {
        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> m_modules.m_frontLeft.getPosition().getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> m_modules.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Front Right Angle", () -> m_modules.m_frontRight.getPosition().getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> m_modules.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Back Left Angle", () -> m_modules.m_backLeft.getPosition().getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> m_modules.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Back Right Angle", () -> m_modules.m_backRight.getPosition().getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> m_modules.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation2D().getRadians(), null);
        });
    }

    /**
     * Runs a System Identification routine for a swerve module
     *
     * @param module       swerve module
     * @param direction    of the module (forward or reverse)
     * @param swerveSubsystem the object of the swerve mechanism
     * @param sysidConfig  routine of the sysid (e.g. step voltage)
     * @param option       either angle or drive routine (different motors!)
     * @param dynamic      is dynamically accelerating or not
     * @return a command of the full routine
     */
    public Command moduleSysidRoutineCommand(SwerveModuleOption module, Direction direction, SubsystemBase swerveSubsystem,
                                             SysidConfig sysidConfig, SysIdRoutineOption option, boolean dynamic) {
        SwerveModule selectedModule;

        switch (module) {
            case FRONT_LEFT -> selectedModule = m_modules.m_frontLeft;
            case FRONT_RIGHT -> selectedModule = m_modules.m_frontRight;
            case BACK_LEFT -> selectedModule = m_modules.m_backLeft;
            case BACK_RIGHT -> selectedModule = m_modules.m_backRight;
            default -> throw new IllegalArgumentException("Invalid module parameter: " + module);
        }

        if (option.equals(SysIdRoutineOption.ANGLE)) {
            if (dynamic)
                return selectedModule.driveSysIdDynamic(direction, swerveSubsystem, sysidConfig);
            return selectedModule.driveSysIdQuas(direction, swerveSubsystem, sysidConfig);
        } else {
            if (dynamic)
                return selectedModule.angleSysIdDynamic(direction, swerveSubsystem, sysidConfig);
            return selectedModule.angleSysIdQuas(direction, swerveSubsystem, sysidConfig);
        }
    }

    /**
     * A method that updates the modules positions of all modules. should be called periodically.
     */
    public void periodic() {
        m_modules.periodic();
    }
}
