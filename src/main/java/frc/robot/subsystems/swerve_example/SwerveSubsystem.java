package frc.robot.subsystems.swerve_example;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.imu.Pigeon;
import frc.excalib.control.math.Vector2D;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.swerve.ModulesHolder;
import frc.excalib.swerve.SwerveMechanism;
import frc.excalib.swerve.SwerveModule;
import frc.excalib.swerve.swerve_utils.SwerveConfigurationUtils;
import frc.excalib.swerve.swerve_utils.SwerveSpecs;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.math.geometry.Rotation2d.kPi;
import static frc.robot.subsystems.swerve_example.Constants.*;

public class SwerveSubsystem extends SubsystemBase implements Logged {
    // An array of the absolute encoders that measure the modules' positions
    private final CANcoder[] m_cancoders;

    // An array of the swerve modules
    private final SwerveModule[] m_swerveModules;

    // The IMU measures the robot's heading
    private final IMU m_gyro;

    // The swerve mechanism
    private final SwerveMechanism m_swerveMechanism;

    // PID controllers for the example turnToAngle and pidToPose commands
    private final PIDController m_angleController;
    private final PIDController m_xController;
    private final PIDController m_yController;
    // setpoint for the example turnToAngle and pidToPose commands
    private Supplier<Rotation2d> m_angleSetpoint;
    private Supplier<Translation2d> m_translationSetpoint;
    // finish trigger for the example pidToPoseCommand.
    private final Trigger m_atPoseTrigger;

    /**
     * A constructor that initialize the swerve subsystem
     */
    public SwerveSubsystem() {
        // The drive motors of the modules.
        // THEY MUST BE ARRANGED IN THE FOLLOWING ORDER: front left, front right, back left, back right
        Motor[] driveMotors = new Motor[]{
                new TalonFXMotor(FRONT_LEFT_DRIVE_ID, SWERVE_CANBUS),
                new TalonFXMotor(FRONT_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                new TalonFXMotor(BACK_LEFT_DRIVE_ID, SWERVE_CANBUS),
                new TalonFXMotor(BACK_RIGHT_DRIVE_ID, SWERVE_CANBUS)
        };

        // The steering motors of the modules.
        // THEY MUST BE ARRANGED IN THE FOLLOWING ORDER: front left, front right, back left, back right
        Motor[] steeringMotors = new Motor[]{
                new SparkMaxMotor(FRONT_LEFT_ROTATION_ID, kBrushless),
                new SparkMaxMotor(FRONT_RIGHT_ROTATION_ID, kBrushless),
                new SparkMaxMotor(BACK_LEFT_ROTATION_ID, kBrushless),
                new SparkMaxMotor(BACK_RIGHT_ROTATION_ID, kBrushless)
        };

        m_cancoders = new CANcoder[]{
                new CANcoder(FRONT_LEFT_CANCODER_ID, SWERVE_CANBUS),
                new CANcoder(FRONT_RIGHT_CANCODER_ID, SWERVE_CANBUS),
                new CANcoder(BACK_LEFT_CANCODER_ID, SWERVE_CANBUS),
                new CANcoder(BACK_RIGHT_CANCODER_ID, SWERVE_CANBUS)
        };

        // An array of DoubleSupplier represents the positions of the modules.
        // THEY MUST BE ARRANGED IN THE FOLLOWING ORDER: front left, front right, back left, back right
        DoubleSupplier[] modulesPositionsSuppliers = {
                () -> m_cancoders[0].getPosition().getValueAsDouble() * ROTATIONS_TO_RADIANS,
                () -> m_cancoders[1].getPosition().getValueAsDouble() * ROTATIONS_TO_RADIANS,
                () -> m_cancoders[2].getPosition().getValueAsDouble() * ROTATIONS_TO_RADIANS,
                () -> m_cancoders[3].getPosition().getValueAsDouble() * ROTATIONS_TO_RADIANS
        };

        // A function that initializes the swerve modules objects
        m_swerveModules = SwerveConfigurationUtils.setupSwerveModules(
                SwerveConfigurationUtils.setupDrivingMechanisms(
                        driveMotors, STALL_DRIVING_MODULE_LIMIT, FREE_DRIVING_MODULE_LIMIT,
                        DirectionState.FORWARD, IdleState.BRAKE, MODULE_TYPE.getModuleConfiguration(),
                        DRIVING_MODULE_GAINS, MAX_MODULE_ACCELERATION, MAX_MODULE_JERK
                ),
                SwerveConfigurationUtils.setupSteeringMechanisms(
                        steeringMotors, STALL_STEERING_MODULE_LIMIT, FREE_STEERING_MODULE_LIMIT,
                        DirectionState.FORWARD, IdleState.BRAKE, MODULE_TYPE.getModuleConfiguration(),
                        STEERING_MODULE_GAINS, modulesPositionsSuppliers, STEERING_PID_TOLERANCE
                ),
                MODULE_LOCATIONS,
                MAX_MODULE_VELOCITY //Optionally, you can set the max velocity of each module separately (using an array of doubles)
        );

        // Initialization of the IMU
        m_gyro = new Pigeon(GYRO_ID, SWERVE_CANBUS, GYRO_OFFSET);

        // Initialization of the swerve mechanism
        m_swerveMechanism = new SwerveMechanism(
                new ModulesHolder(
                        // MUST BE ARRANGED IN THE FOLLOWING ORDER: front left, front right, back left, back right
                        m_swerveModules[0],
                        m_swerveModules[1],
                        m_swerveModules[2],
                        m_swerveModules[3]
                ),
                new SwerveSpecs(MAX_VELOCITY, MAX_OMEGA_RAD_PER_SEC, MAX_ACC, MAX_FRONT_ACC, MAX_SIDE_ACC, MAX_FORWARD_ACC),
                m_gyro,
                INITIAL_SWERVE_POSITION
        );

        // Initialization of the PID controllers
        m_angleController = new PIDController(ANGLE_PID_GAINS.kp, ANGLE_PID_GAINS.ki, ANGLE_PID_GAINS.kd);
        m_xController = new PIDController(TRANSLATION_PID_GAINS.kp, TRANSLATION_PID_GAINS.ki, TRANSLATION_PID_GAINS.kd);
        m_yController = new PIDController(TRANSLATION_PID_GAINS.kp, TRANSLATION_PID_GAINS.ki, TRANSLATION_PID_GAINS.kd);

        // Initialization of the setpoints
        m_angleSetpoint = Rotation2d::new;
        m_translationSetpoint = Translation2d::new;

        // Initialization of the finish trigger
        m_atPoseTrigger = new Trigger(m_xController::atSetpoint).and(m_yController::atSetpoint).and(m_angleController::atSetpoint).debounce(0.1);

        // Initialization of the AutoBuilder for pathplanner
         initAutoBuilder();
    }

    /**
     * Takes the drive command from the swerve mechanism.
     *
     * @param velocityMPS    Supplier for the desired linear velocity in meters per second.
     * @param omegaRadPerSec Supplier for the desired angular velocity in radians per second.
     * @param fieldOriented  Supplier indicating whether the control is field-oriented.
     * @return A command that drives the robot.
     */
    public Command driveCommand(Supplier<Vector2D> velocityMPS,
                                DoubleSupplier omegaRadPerSec,
                                BooleanSupplier fieldOriented) {
        return m_swerveMechanism.driveCommand(velocityMPS, omegaRadPerSec, fieldOriented, true, this).withName("Drive Command");
    }

    /**
     * Turns the robot to a desired angle.
     *
     * @param angleSetpoint The desired angle in radians.
     * @return A command that turns the robot to the wanted angle.
     */
    public Command turnToAngleCommand(Supplier<Vector2D> velocityMPS, Supplier<Rotation2d> angleSetpoint) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    m_angleController.calculate(m_swerveMechanism.getRotation2D().getRadians(), angleSetpoint.get().getRadians());
                    m_angleSetpoint = angleSetpoint;
                }),
                driveCommand(
                        velocityMPS,
                        () -> m_angleController.calculate(m_swerveMechanism.getRotation2D().getRadians(), angleSetpoint.get().getRadians()),
                        () -> true
                )
        ).until(m_angleController::atSetpoint).withName("Turn To Angle Command");
    }

    /**
     * Drives the robot to a desired pose.
     *
     * @param poseSetpoint The desired pose on the field.
     * @return A command that turns the robot to the wanted angle.
     */
    public Command pidToPoseCommand(Supplier<Pose2d> poseSetpoint) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            m_xController.calculate(m_swerveMechanism.getPose2D().getX(), poseSetpoint.get().getX());
                            m_yController.calculate(m_swerveMechanism.getPose2D().getY(), poseSetpoint.get().getY());
                            m_angleController.calculate(m_swerveMechanism.getRotation2D().getRadians(), poseSetpoint.get().getRotation().getRadians());
                            m_translationSetpoint = () -> poseSetpoint.get().getTranslation();
                            m_angleSetpoint = () -> poseSetpoint.get().getRotation();
                        }
                ),
                driveCommand(
                        () -> {
                            Vector2D vel = new Vector2D(
                                    m_xController.calculate(m_swerveMechanism.getPose2D().getX(), poseSetpoint.get().getX()),
                                    m_yController.calculate(m_swerveMechanism.getPose2D().getY(), poseSetpoint.get().getY())
                            );
                            if (AllianceUtils.isRedAlliance()) return vel.rotate(kPi);
                            return vel;
                        },
                        () -> m_angleController.calculate(m_swerveMechanism.getRotation2D().getRadians(), poseSetpoint.get().getRotation().getRadians()),
                        () -> true
                )
        ).until(m_atPoseTrigger).withName("PID To Pose Command");
    }

    /**
     * A function that initialize the AutoBuilder for pathplanner.
     */
    public void initAutoBuilder() {
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
        AutoBuilder.configure(
                m_swerveMechanism::getPose2D, // Robot pose supplier
                m_swerveMechanism::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                m_swerveMechanism::getRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> m_swerveMechanism.driveRobotRelativeChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        TRANSLATION_PID_PP_CONSTANTS, // Translation PID constants
                        ANGLE_PID_PP_CONSTANTS // Rotation PID constants
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
     * A method that updates the modules positions of all modules.
     */
    @Override
    public void periodic() {
        m_swerveMechanism.periodic();

        m_swerveMechanism.updateOdometry();
    }
}
