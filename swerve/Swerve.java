package frc.excalib.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.additional_utilities.Elastic;
import frc.excalib.slam.mapper.Odometry;
import frc.excalib.control.math.Vector2D;
import frc.excalib.control.gains.SysidConfig;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import monologue.Logged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.kTextView;
import static frc.excalib.additional_utilities.Elastic.Notification.NotificationLevel.WARNING;
import static frc.robot.Constants.SwerveConstants.*;
import static monologue.Annotations.Log;

/**
 * A class representing a swerve subsystem.
 */
public class Swerve extends SubsystemBase implements Logged {
    public final ModulesHolder modules;
    public final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    public final Odometry m_odometry;
    private ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();
    private Trigger finishTrigger;
    private Rotation2d pi = new Rotation2d(Math.PI);

    private final SwerveDriveKinematics m_swerveDriveKinematics;
    private final PIDController angleController = new PIDController(ANGLE_PID_GAINS.kp, ANGLE_PID_GAINS.ki, ANGLE_PID_GAINS.kd);
    private final PIDController xController = new PIDController(
            TRANSLATION_PID_GAINS.kp, TRANSLATION_PID_GAINS.ki, TRANSLATION_PID_GAINS.kd
    );
    private final PIDController yController = new PIDController(
            TRANSLATION_PID_GAINS.kp, TRANSLATION_PID_GAINS.ki, TRANSLATION_PID_GAINS.kd
    );
    public final Field2d field = new Field2d();

    // AdvantageKit logging inputs for Swerve (pose + chassis speeds)
    private final SwerveInputsAutoLogged akInputs = new SwerveInputsAutoLogged();

    private Supplier<Rotation2d> angleSetpoint = Rotation2d::new;
    private Supplier<Translation2d> m_translationSetpoint = Translation2d::new;



    /**
     * A constructor that initialize the Swerve Subsystem
     *
     * @param modules         The ModulesHolder containing all swerve modules.
     * @param gyroIO          GyroIO sensor.
     * @param initialPosition The initial position of the robot.
     */
    public Swerve(ModulesHolder modules,
                  GyroIO gyroIO,
                  Pose2d initialPosition) {
        this.modules = modules;
        this.gyroIO = gyroIO;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        angleController.setTolerance(0.0628);

        finishTrigger = new Trigger(xController::atSetpoint).and(yController::atSetpoint).and(angleController::atSetpoint).debounce(0.04);
        // Initialize odometry with the current yaw angle
        this.m_odometry = new Odometry(
                modules.getSwerveDriveKinematics(),
                modules.getModulesPositions(),
                () -> Rotation2d.fromRadians(gyroInputs.yawPositionRad),
                initialPosition
        );

        m_swerveDriveKinematics = this.modules.getSwerveDriveKinematics();

        initAutoBuilder();
        initElastic();
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
            Supplier<Vector2D> velocityMPS, DoubleSupplier omegaRadPerSec, BooleanSupplier fieldOriented) {

        // Precompute values to avoid redundant calculations
        Supplier<Vector2D> adjustedVelocitySupplier = () -> {
            Vector2D velocity = velocityMPS.get();
//            Vector2D velocity = SwerveAccUtils.getSmartTranslationalVelocitySetpoint(getVelocity(), velocityMPS.get());
            if (fieldOriented.getAsBoolean()) {
                Rotation2d yaw = getRotation2D().unaryMinus();
                if (!AllianceUtils.isBlueAlliance()) yaw = yaw.plus(pi);
                return velocity.rotate(yaw);
            }
            return velocity;
        };

        Command driveCommand = new ParallelCommandGroup(
                modules.setVelocitiesCommand(
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
        modules.setModulesStates(m_swerveDriveKinematics.toSwerveModuleStates(speeds));
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
                            m_translationSetpoint = () -> poseSetpoint.get().getTranslation();
                            angleSetpoint = () -> poseSetpoint.get().getRotation();
                        }
                ),
                driveCommand(
                        () -> {
                            Vector2D vel = new Vector2D(
                                    xController.calculate(getPose2D().getX(), poseSetpoint.get().getX()),
                                    yController.calculate(getPose2D().getY(), poseSetpoint.get().getY())
                            );
//                            System.out.println("current:  " + getRotation2D().getRadians());
//                            System.out.println("output:  " + angleController.calculate(getRotation2D().getRadians(), poseSetpoint.get().getRotation().getRadians()));
//                            System.out.println("error:  " + angleController.getError());
                            if (!AllianceUtils.isBlueAlliance()) return vel.rotate(pi);
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
        return new InstantCommand(() -> gyroIO.getIMU().resetIMU()).ignoringDisable(true);
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
        m_odometry.updateOdometry(modules.getModulesPositions());
    }

    /**
     * A method that restarts the odometry.
     *
     * @param newPose the wanted new Pose2d of the robot.
     */
    public void resetOdometry(Pose2d newPose) {
        m_odometry.resetOdometry(modules.getModulesPositions(), newPose);
    }

    public Command resetOdometryCommand(Pose2d newPose) {
        return new InstantCommand(() -> m_odometry.resetOdometry(modules.getModulesPositions(), newPose));
    }

    /**
     * Gets the robot's rotation.
     *
     * @return The current rotation of the robot.
     */
    @Log.NT(key = "Robot Rotation")
    public Rotation2d getRotation2D() {
        return Rotation2d.fromRadians(gyroInputs.yawPositionRad);
    }

    @Log.NT(key = "Angle Setpoint")
    public Rotation2d getAngleSetpoint() {
        return angleSetpoint.get();
    }

    @Log.NT(key = "Translation Setpoint")
    public Translation2d getTranslationSetpoint() {
        return m_translationSetpoint.get();
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

    @AutoLogOutput(key = "Swerve/PoseX")
    @Log.NT
    public double getPoseX() {
        return getPose2D().getX();
    }

    @AutoLogOutput(key = "Swerve/PoseY")
    @Log.NT
    public double getPoseY() {
        return getPose2D().getY();
    }

    @AutoLogOutput(key = "Swerve/PoseTheta")
    @Log.NT
    public double getPoseTheta() {
        return getPose2D().getRotation().getRadians();
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
    public double getAccelerationDistance() {
        return new Vector2D(gyroInputs.accelX, gyroInputs.accelY).getDistance();
    }

    /**
     * Gets the current robot relative speed.
     *
     * @return The robot's speed as a ChassisSpeeds.
     */
    @Log.NT(key = "Measured Chassis Speeds")
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_swerveDriveKinematics.toChassisSpeeds(modules.logStates());
    }

    @Log.NT
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return m_desiredChassisSpeeds;
    }

    @AutoLogOutput(key = "Swerve/MeasuredVx")
    @Log.NT
    public double getMeasuredVx() {
        return getRobotRelativeSpeeds().vxMetersPerSecond;
    }

    @AutoLogOutput(key = "Swerve/MeasuredVy")
    @Log.NT
    public double getMeasuredVy() {
        return getRobotRelativeSpeeds().vyMetersPerSecond;
    }

    @AutoLogOutput(key = "Swerve/MeasuredOmega")
    @Log.NT
    public double getMeasuredOmega() {
        return getRobotRelativeSpeeds().omegaRadiansPerSecond;
    }

    @AutoLogOutput(key = "Swerve/DesiredVx")
    @Log.NT
    public double getDesiredVx() {
        return getDesiredChassisSpeeds().vxMetersPerSecond;
    }

    @AutoLogOutput(key = "Swerve/DesiredVy")
    @Log.NT
    public double getDesiredVy() {
        return getDesiredChassisSpeeds().vyMetersPerSecond;
    }

    @AutoLogOutput(key = "Swerve/DesiredOmega")
    @Log.NT
    public double getDesiredOmega() {
        return getDesiredChassisSpeeds().omegaRadiansPerSecond;
    }

    /// /    public double distanceFromReefCenter() {
    /// /        return AllianceUtils.isBlueAlliance() ?
    /// /                BLUE_REEF_CENTER.getDistance(getPose2D().getTranslation()) :
    /// /                RED_REEF_CENTER.getDistance(getPose2D().getTranslation());
    /// /    }

    public Command stopCommand() {
        return driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true);
    }

    /**
     * Test command to drive the robot straight at 1.0 m/s.
     * Useful for verifying simulation physics without a controller.
     */
    public Command driveStraightCommand() {
        return driveCommand(() -> new Vector2D(1.0, 0), () -> 0, () -> true).withName("Drive Straight Test");
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
            System.out.println("the config is null");
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose2D, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelativeChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        TRANSLATION_PID_PP_CONSTANTS, // Translation PID constants
                        ANGLE_PID_PP_CONSTANTS // Rotation PID constants
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * A function that initialize the Swerve tab for Elastic.
     */
    public void initElastic() {
        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> modules.m_frontLeft.getPosition().getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> modules.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Front Right Angle", () -> modules.m_frontRight.getPosition().getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> modules.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Back Left Angle", () -> modules.m_backLeft.getPosition().getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> modules.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Back Right Angle", () -> modules.m_backRight.getPosition().getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> modules.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation2D().getRadians(), null);
        });

        SmartDashboard.putData("Field", field);

        SmartDashboard.putData("swerve info", this);

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

        GenericEntry odometryXEntry = swerveTab.add("odometryX", 0).withWidget(kTextView).getEntry();
        GenericEntry odometryYEntry = swerveTab.add("odometryY", 0).withWidget(kTextView).getEntry();
        GenericEntry odometryAngleEntry = swerveTab.add("odometryAngle", 0).withWidget(kTextView).getEntry();

        swerveTab.add("Reset Odometry",
                new InstantCommand(
                        () -> resetOdometry(
                                new Pose2d(
                                        odometryXEntry.getDouble(0),
                                        odometryYEntry.getDouble(0),
                                        Rotation2d.fromDegrees(odometryAngleEntry.getDouble(0)))
                        ), this).ignoringDisable(true)
        );

        GenericEntry OTFGxEntry = swerveTab.add("OTFGx", 0).withWidget(kTextView).getEntry();
        GenericEntry OTFGyEntry = swerveTab.add("OTFGy", 0).withWidget(kTextView).getEntry();
        GenericEntry OTFGAngleEntry = swerveTab.add("OTFGAngle", 0).withWidget(kTextView).getEntry();
        swerveTab.add("Drive To Pose",
                driveToPoseCommand(
                        new Pose2d(
                                OTFGxEntry.getDouble(0),
                                OTFGyEntry.getDouble(0),
                                Rotation2d.fromDegrees(OTFGAngleEntry.getDouble(0)))
                )
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

    @Override
    public void periodic() {
        modules.periodic();
        
        gyroIO.updateInputs(gyroInputs);
        org.littletonrobotics.junction.Logger.processInputs("Swerve/Gyro", gyroInputs);

        field.setRobotPose(getPose2D());
        updateOdometry();
        
        /* 
         * Commented out vision-based reset to prevent perpetual "teleportation" to vision pose.
         * Vision should either be added via addVisionMeasurement or only used for the first init.
        Pose2d arrPose = getAuroraPose2d();
        if (!((arrPose.getX() == 0) && (arrPose.getY() == 0) && (arrPose.getRotation().getRadians() == 0))) {
            m_odometry.resetOdometry(modules.getModulesPositions(), getAuroraPose2d());
        }
        */
        // Populate AdvantageKit inputs and pass them to the Logger as a grouped input
        // object. This gives a single table in the .wpilog and AdvantageScope called
        // "Swerve" containing pose + measured/desired chassis speeds.
        akInputs.poseX = getPose2D().getX();
        akInputs.poseY = getPose2D().getY();
        akInputs.poseTheta = getPose2D().getRotation().getRadians();

        ChassisSpeeds measured = getRobotRelativeSpeeds();
        akInputs.measuredVx = measured.vxMetersPerSecond;
        akInputs.measuredVy = measured.vyMetersPerSecond;
        akInputs.measuredOmega = measured.omegaRadiansPerSecond;

        ChassisSpeeds desired = getDesiredChassisSpeeds();
        akInputs.desiredVx = desired.vxMetersPerSecond;
        akInputs.desiredVy = desired.vyMetersPerSecond;
        akInputs.desiredOmega = desired.omegaRadiansPerSecond;

        org.littletonrobotics.junction.Logger.processInputs("Swerve", akInputs);

        // === SWERVE CHASSIS DIAGNOSTICS (~1/sec) ===
        if (Math.random() < 0.02) {
            System.out.println("===== SWERVE CHASSIS DIAGNOSTICS =====");
            System.out.printf("  Gyro Yaw:       %.3f rad (%.1f deg)%n", gyroInputs.yawPositionRad, Math.toDegrees(gyroInputs.yawPositionRad));
            System.out.printf("  Desired:        Vx=%.2f  Vy=%.2f  Omega=%.2f%n", desired.vxMetersPerSecond, desired.vyMetersPerSecond, desired.omegaRadiansPerSecond);
            System.out.printf("  Measured:       Vx=%.2f  Vy=%.2f  Omega=%.2f%n", measured.vxMetersPerSecond, measured.vyMetersPerSecond, measured.omegaRadiansPerSecond);
            System.out.printf("  Pose:           X=%.2f  Y=%.2f  Theta=%.1f deg%n", akInputs.poseX, akInputs.poseY, Math.toDegrees(akInputs.poseTheta));
            edu.wpi.first.math.kinematics.SwerveModuleState[] states = modules.logStates();
            for (int i = 0; i < states.length; i++) {
                System.out.printf("  Module %d:       speed=%.2f m/s  angle=%.1f deg%n", i, states[i].speedMetersPerSecond, states[i].angle.getDegrees());
            }
            System.out.println("======================================");
        }

        // Standard AdvantageKit Logging Pattern for Swerve States (AdvantageScope)
        org.littletonrobotics.junction.Logger.recordOutput("SwerveStates/Measured", modules.logStates());
        org.littletonrobotics.junction.Logger.recordOutput("SwerveStates/Setpoints", modules.logSetPointStates());
        org.littletonrobotics.junction.Logger.recordOutput("Odometry/Robot", getPose2D());
    }

    @Log.NT
    public Pose2d getAuroraPose2d() {
        Pose2d auroraPose = new Pose2d(
                new Translation2d(
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("x").getDouble(0),
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("y").getDouble(0)
                ),
                new Rotation2d(NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("yaw").getDouble(0)
                )
        );

        return auroraPose;
    }

    @Log.NT
    public Pose3d getAuroraPose3() {
        Pose3d auroraPose = new Pose3d(
                new Translation3d(
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("x").getDouble(0),
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("y").getDouble(0),
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("z").getDouble(0)
                ),
                new Rotation3d(
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("roll").getDouble(0),
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("pitch").getDouble(0),
                        NetworkTableInstance.getDefault().getTable("Aurora").getSubTable("robotPose").getEntry("yaw").getDouble(0)
                )
        );

        return auroraPose;
    }

    ;

    @Log.NT
    public boolean isAtPosition() {
        return finishTrigger.getAsBoolean();
    }

    @Log.NT
    public boolean isAtxPosition() {
        return xController.atSetpoint();
    }

    @Log.NT
    public boolean isAtyPosition() {
        return yController.atSetpoint();
    }

    @Log.NT
    public boolean isAtAnglePosition() {
        return angleController.atSetpoint();
    }

}