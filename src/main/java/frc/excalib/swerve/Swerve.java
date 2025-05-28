package frc.excalib.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.additional_utilities.Elastic;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.math.Vector2D;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.slam.mapper.Odometry;
import monologue.Logged;
import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.apriltag.AprilTagFields.*;
import static frc.excalib.additional_utilities.Elastic.Notification.NotificationLevel.WARNING;
import static frc.robot.SwerveConstants.*;
import static monologue.Annotations.Log;

/**
 * A class representing a swerve subsystem.
 */
public class Swerve extends SubsystemBase implements Logged {
    private final ModulesHolder modules;
    private final IMU imu;
    private final Odometry odometry;
    PhotonAprilTagsCamera exampleCamera;
    private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();
    private final Trigger finishTrigger;
    private final Rotation2d PI = new Rotation2d(Math.PI);
    private final InterpolatingDoubleTreeMap velocityLimit = new InterpolatingDoubleTreeMap();

    private final SwerveDriveKinematics swerveDriveKinematics;

    private final PIDController angleController = new PIDController(ANGLE_GAINS.kp, ANGLE_GAINS.ki, ANGLE_GAINS.kd);
    private final PIDController xController = new PIDController(TRANSLATION_GAINS.kp, TRANSLATION_GAINS.ki, TRANSLATION_GAINS.kd);
    private final PIDController yController = new PIDController(TRANSLATION_GAINS.kp, TRANSLATION_GAINS.ki, TRANSLATION_GAINS.kd);
    public final Field2d field = new Field2d();
    private Supplier<Rotation2d> angleSetpoint = Rotation2d::new;
    private Supplier<Translation2d> translationSetpoint = Translation2d::new;

    /**
     * A constructor that initialize the Swerve Subsystem
     *
     * @param modules         The ModulesHolder containing all swerve modules.
     * @param imu             IMU sensor.
     * @param initialPosition The initial position of the robot.
     */
    public Swerve(ModulesHolder modules, IMU imu, Pose2d initialPosition) {
        this.modules = modules;
        this.imu = imu;
        this.imu.resetIMU();

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(ANGLE_CONTROLLER_TOLORANCE);
        xController.setTolerance(X_CONTROLLER_TOLORANCE);
        yController.setTolerance(Y_CONTROLLER_TOLORANCE);

        finishTrigger = new Trigger(xController::atSetpoint).and(yController::atSetpoint).and(angleController::atSetpoint).debounce(0.1);
        this.odometry = new Odometry(modules.getSwerveDriveKinematics(), modules.getModulesPositions(), this.imu::getZRotation, initialPosition);
        PhotonAprilTagsCamera m_exampleCamera = new PhotonAprilTagsCamera("example", k2025ReefscapeWelded, new Transform3d(0, 0, 0, new Rotation3d()));

        swerveDriveKinematics = this.modules.getSwerveDriveKinematics();
        velocityLimit.put(0.1, 0.4);
        velocityLimit.put(0.7, 2.0);
        velocityLimit.put(1.5, MAX_VEL);

        initAutoBuilder();
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
            BooleanSupplier fieldOriented) {

        // Precompute values to avoid redundant calculations
        Supplier<Vector2D> adjustedVelocitySupplier = () -> {
            Vector2D velocity = velocityMPS.get();
//            Vector2D velocity = getSmartTranslationalVelocitySetPoint(getVelocity(), velocityMPS.get());
            if (fieldOriented.getAsBoolean()) {
                Rotation2d yaw = getRotation2D().unaryMinus();
                if (!AllianceUtils.isBlueAlliance()) yaw = yaw.plus(PI);
                return velocity.rotate(yaw);
            }
            return velocity;
        };

        Command driveCommand = new ParallelCommandGroup(modules.setVelocitiesCommand(
                adjustedVelocitySupplier,
                omegaRadPerSec
        ),
                new RunCommand(
                        () -> desiredChassisSpeeds = new ChassisSpeeds(
                                adjustedVelocitySupplier.get().getX(),
                                adjustedVelocitySupplier.get().getY(),
                                omegaRadPerSec.getAsDouble())
                )
        );
        driveCommand.setName("Drive Command");
        driveCommand.addRequirements(this);
        return driveCommand;
    }

    /**
     * A non-command function that drives the robot (for pathplanner).
     *
     * @param speeds A ChassisSpeeds object represents ROBOT RELATIVE speeds desired speeds.
     */
    public void driveRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        modules.setModulesStates(swerveDriveKinematics.toSwerveModuleStates(speeds));
    }

    /**
     * A method that turns the robot to a desired angle.
     *
     * @param angleSetpoint The desired angle in radians.
     * @return A command that turns the robot to the wanted angle.
     */
    public Command turnToAngleCommand(Supplier<Vector2D> velocityMPS, Supplier<Rotation2d> angleSetpoint) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> this.angleSetpoint = angleSetpoint),
                driveCommand(
                        velocityMPS,
                        () -> angleController.calculate(getRotation2D().getRadians(), angleSetpoint.get().getRadians()),
                        () -> true
                )
        ).withName("Turn To Angle");
    }

    public Command pidToPoseCommand(Supplier<Pose2d> poseSetpoint) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            xController.calculate(getPose2D().getX(), poseSetpoint.get().getX());
                            yController.calculate(getPose2D().getY(), poseSetpoint.get().getY());
                            angleController.calculate(getRotation2D().getRadians(), poseSetpoint.get().getRotation().getRadians());
                            translationSetpoint = () -> poseSetpoint.get().getTranslation();
                            angleSetpoint = () -> poseSetpoint.get().getRotation();
                        }
                ),
                driveCommand(
                        () -> {
                            Vector2D vel = new Vector2D(
                                    xController.calculate(getPose2D().getX(), poseSetpoint.get().getX()),
                                    yController.calculate(getPose2D().getY(), poseSetpoint.get().getY())
                            );
                            double distance = getPose2D().getTranslation().getDistance(poseSetpoint.get().getTranslation());
                            vel.setMagnitude(Math.min(vel.getDistance(), velocityLimit.get(distance)));
//                            vel = vel.rotate(poseSetpoint.get().getRotation());
//                            vel.setX(Math.signum(vel.getX()) * Math.min(Math.abs(vel.getX()), 1.2));
//                            vel.setY(Math.signum(vel.getY()) * Math.min(Math.abs(vel.getY()), 0.4));
//                            vel = vel.rotate(poseSetpoint.get().getRotation().unaryMinus());
                            if (!AllianceUtils.isBlueAlliance()) return vel.rotate(PI);
                            return vel;
                        },
                        () -> angleController.calculate(getRotation2D().getRadians(), poseSetpoint.get().getRotation().getRadians()),
                        () -> true
                )
        ).until(finishTrigger).withName("PID To Pose");
    }

    /**
     * A method that drives the robot to a desired pose.
     *
     * @param setPoint The desired pose.
     * @return A command that drives the robot to the wanted pose.
     */
    public Command driveToPoseCommand(Pose2d setPoint) {
        return AutoBuilder.pathfindToPose(
                setPoint,
                MAX_PATH_CONSTRAINTS
        ).withName("Pathfinding Command");
    }

    public Command driveToPoseWithOverrideCommand(
            Pose2d setPoint,
            BooleanSupplier override,
            Supplier<Vector2D> velocityMPS,
            DoubleSupplier omegaRadPerSec) {
        Command driveToPoseCommand = driveToPoseCommand(setPoint);
        return new SequentialCommandGroup(
                driveToPoseCommand.until(() -> velocityMPS.get().getDistance() != 0 && override.getAsBoolean()),
                driveCommand(
                        velocityMPS,
                        omegaRadPerSec,
                        () -> true
                ).until(() -> velocityMPS.get().getDistance() == 0)
        ).repeatedly().until(driveToPoseCommand::isFinished).withName("Pathfinding With Override Command");
    }

    /**
     * A method that drives the robot to the starting pose of a path, then follows the path.
     *
     * @param pathName The path which the robot needs to follow.
     * @return A command that turns the robot to the wanted angle.
     */
    public Command pathfindThenFollowPathCommand(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            Elastic.sendNotification(new Elastic.Notification(
                    WARNING,
                    "Path Creating Error",
                    "the path file " + pathName + " doesn't exist")
            );
            return new PrintCommand("this path file doesn't exist");
        }

        return AutoBuilder.pathfindThenFollowPath(
                path,
                MAX_PATH_CONSTRAINTS
        );
    }

    public Command resetAngleCommand() {
        return new InstantCommand(imu::resetIMU).ignoringDisable(true);
    }

    public Command coastCommand() {
        Command coastCommand = modules.coastCommand().ignoringDisable(true).withName("Coast Command");
        coastCommand.addRequirements(this);
        return coastCommand;
    }

    /**
     * Updates the robot's odometry.
     */
    public void updateOdometry() {
        odometry.updateOdometry(modules.getModulesPositions());
        Optional<EstimatedRobotPose> backPose = exampleCamera.getEstimatedGlobalPose(odometry.getEstimatedPosition());
        backPose.ifPresent(
                estimatedRobotPose -> odometry.addVisionMeasurement(
                        estimatedRobotPose.estimatedPose.toPose2d(),
                        estimatedRobotPose.timestampSeconds
                )
        );
    }

    /**
     * A method that restarts the odometry.
     *
     * @param newPose the wanted new Pose2d of the robot.
     */
    public void resetOdometry(Pose2d newPose) {
        odometry.resetOdometry(modules.getModulesPositions(), newPose);
    }

    /**
     * Gets the robot's rotation.
     *
     * @return The current rotation of the robot.
     */
    @Log.NT(key = "Robot Rotation")
    public Rotation2d getRotation2D() {
        return getPose2D().getRotation();
    }

    @Log.NT(key = "Angle Setpoint")
    public Rotation2d getAngleSetpoint() {
        return angleSetpoint.get();
    }

    @Log.NT(key = "Translation Setpoint")
    public Translation2d getTranslationSetpoint() {
        return translationSetpoint.get();
    }

    /**
     * Gets the robot's pose.
     *
     * @return The current pose of the robot.
     */
    @Log.NT(key = "Robot Pose")
    public Pose2d getPose2D() {
        return odometry.getRobotPose();
    }

    /**
     * Gets the current velocity of the robot.
     *
     * @return The robot's velocity as a Vector2D.
     */
    public Vector2D getVelocity() {
        return modules.getVelocity();
    }

    /**
     * Gets the current acceleration of the robot.
     *
     * @return The robot's acceleration as a Vector2D.
     */
    @Log.NT(key = "Acceleration")
    public double getAccelerationDistance() {
        return new Vector2D(imu.getAccX(), imu.getAccY()).getDistance();
    }

    /**
     * Gets the current robot relative speed.
     *
     * @return The robot's speed as a ChassisSpeeds.
     */
    @Log.NT(key = "Measured Chassis Speeds")
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDriveKinematics.toChassisSpeeds(modules.logStates());
    }

    @Log.NT
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return desiredChassisSpeeds;
    }

    public Command stopCommand() {
        return driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true);
    }

    /**
     * A function that initialize the AutoBuilder for pathplanner.
     */
    private void initAutoBuilder() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        assert config != null;
        AutoBuilder.configure(
                this::getPose2D, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelativeChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        TRANSLATION_PID_CONSTANTS, // Translation PID constants
                        ANGLE_PID_CONSTANTS // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );
    }


    /**
     * Runs a system identification routine on a specific module's angle.
     *
     * @param module  The module index (0-3).
     * @param dir     The direction of the sysid routine.
     * @param dynamic Whether to perform a dynamic or quasistatic test.
     * @return The command to perform the sysid routine.
     */
    public Command driveSysId(int module, Direction dir, SysidConfig sysidConfig, boolean dynamic) {
        SwerveModule selectedModule;

        switch (module) {
            case 0 -> selectedModule = modules.m_frontLeft;
            case 1 -> selectedModule = modules.m_frontRight;
            case 2 -> selectedModule = modules.m_backLeft;
            case 3 -> selectedModule = modules.m_backRight;
            default -> {
                throw new IllegalArgumentException("Invalid module index: " + module);
            }
        }
        return dynamic ?
                selectedModule.driveSysIdDynamic(dir, this, sysidConfig)
                : selectedModule.driveSysIdQuas(dir, this, sysidConfig);
    }

    /**
     * Runs a system identification routine on a specific module's angle.
     *
     * @param module  The module index (0-3).
     * @param dir     The direction of the sysid routine.
     * @param dynamic Whether to perform a dynamic or quasistatic test.
     * @return The command to perform the sysid routine.
     */
    public Command angleSysId(int module, Direction dir, SysidConfig sysidConfig, boolean dynamic) {
        SwerveModule selectedModule;

        switch (module) {
            case 0 -> selectedModule = modules.m_frontLeft;
            case 1 -> selectedModule = modules.m_frontRight;
            case 2 -> selectedModule = modules.m_backLeft;
            case 3 -> selectedModule = modules.m_backRight;
            default -> {
                throw new IllegalArgumentException("Invalid module index: " + module);
            }
        }
        return dynamic ?
                selectedModule.angleSysIdDynamic(dir, this, sysidConfig)
                : selectedModule.angleSysIdQuas(dir, this, sysidConfig);
    }

    public static Swerve configureSwerve(Pose2d initialPose) {
        return new Swerve(
                new ModulesHolder(
                        new SwerveModule(
                                new TalonFXMotor(FRONT_LEFT_DRIVE_ID, SWERVE_CANBUS),
                                new SparkMaxMotor(FRONT_LEFT_ROTATION_ID, kBrushless),
                                SWERVE_DRIVE_MODULE_GAINS,
                                SWERVE_ROTATION_MODULE_GAINS,
                                PID_TOLERANCE,
                                FRONT_LEFT_TRANSLATION,
                                () -> FRONT_LEFT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                MAX_MODULE_VEL,
                                VELOCITY_CONVERSION_FACTOR,
                                POSITION_CONVERSION_FACTOR,
                                ROTATION_VELOCITY_CONVERSION_FACTOR
                        ),
                        new SwerveModule(
                                new TalonFXMotor(FRONT_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                                new SparkMaxMotor(FRONT_RIGHT_ROTATION_ID, kBrushless),
                                SWERVE_DRIVE_MODULE_GAINS,
                                SWERVE_ROTATION_MODULE_GAINS,
                                PID_TOLERANCE,
                                FRONT_RIGHT_TRANSLATION,
                                () -> FRONT_RIGHT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                MAX_MODULE_VEL,
                                VELOCITY_CONVERSION_FACTOR,
                                POSITION_CONVERSION_FACTOR,
                                ROTATION_VELOCITY_CONVERSION_FACTOR
                        ),
                        new SwerveModule(
                                new TalonFXMotor(BACK_LEFT_DRIVE_ID, SWERVE_CANBUS),
                                new SparkMaxMotor(BACK_LEFT_ROTATION_ID, kBrushless),
                                SWERVE_DRIVE_MODULE_GAINS,
                                SWERVE_ROTATION_MODULE_GAINS,
                                PID_TOLERANCE,
                                BACK_LEFT_TRANSLATION,
                                () -> BACK_LEFT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                MAX_MODULE_VEL,
                                VELOCITY_CONVERSION_FACTOR,
                                POSITION_CONVERSION_FACTOR,
                                ROTATION_VELOCITY_CONVERSION_FACTOR
                        ),
                        new SwerveModule(
                                new TalonFXMotor(BACK_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                                new SparkMaxMotor(BACK_RIGHT_ROTATION_ID, kBrushless),
                                SWERVE_DRIVE_MODULE_GAINS,
                                SWERVE_ROTATION_MODULE_GAINS,
                                PID_TOLERANCE,
                                BACK_RIGHT_TRANSLATION,
                                () -> BACK_RIGHT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                MAX_MODULE_VEL,
                                VELOCITY_CONVERSION_FACTOR,
                                POSITION_CONVERSION_FACTOR,
                                ROTATION_VELOCITY_CONVERSION_FACTOR
                        )),
                GYRO,
                initialPose
        );
    }

    @Override
    public void periodic() {
        modules.periodic();
        field.setRobotPose(getPose2D());
    }
}
