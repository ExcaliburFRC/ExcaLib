package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import static frc.robot.subsystems.swerve.Constants.*;

public class SwerveSubsystem extends SubsystemBase implements Logged {
    private final CANcoder[] m_cancoders;

    private final SwerveModule[] m_swerveModules;
    private final SwerveMechanism m_swerveMechanism;
    private final IMU m_gyro;

    private final PIDController m_angleController;
    private final PIDController m_xController;
    private final PIDController m_yController;

    private Supplier<Rotation2d> m_angleSetpoint;
    private Supplier<Translation2d> m_translationSetpoint;

    private final Trigger m_finishTrigger;

    public SwerveSubsystem() {
        Motor[] driveMotors = new Motor[]{
                new TalonFXMotor(FRONT_LEFT_DRIVE_ID, SWERVE_CANBUS),
                new TalonFXMotor(FRONT_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                new TalonFXMotor(BACK_LEFT_DRIVE_ID, SWERVE_CANBUS),
                new TalonFXMotor(BACK_RIGHT_DRIVE_ID, SWERVE_CANBUS)
        };

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

        DoubleSupplier[] steeringPositionSuppliers = {
                () -> m_cancoders[0].getPosition().getValueAsDouble(),
                () -> m_cancoders[1].getPosition().getValueAsDouble(),
                () -> m_cancoders[2].getPosition().getValueAsDouble(),
                () -> m_cancoders[3].getPosition().getValueAsDouble()
        };

        m_swerveModules = SwerveConfigurationUtils.setupSwerveModules(
                SwerveConfigurationUtils.setupDrivingMechanisms(
                        driveMotors, STALL_DRIVING_MODULE_LIMIT, FREE_DRIVING_MODULE_LIMIT,
                        DirectionState.FORWARD, IdleState.BRAKE, MODULE_TYPE,
                        DRIVING_MODULE_GAINS, MAX_MODULE_ACCELERATION, MAX_MODULE_JERK
                ),
                SwerveConfigurationUtils.setupSteeringMechanisms(
                        steeringMotors, STALL_STEERING_MODULE_LIMIT, FREE_STEERING_MODULE_LIMIT,
                        DirectionState.FORWARD, IdleState.BRAKE, MODULE_TYPE,
                        STEERING_MODULE_GAINS, steeringPositionSuppliers, STEERING_PID_TOLERANCE
                ),
                MODULE_LOCATIONS,
                MAX_MODULE_VELOCITY //Optionally, you can set the max velocity of each module separately (using an array of doubles)
        );

        m_gyro = new Pigeon(GYRO_ID, SWERVE_CANBUS, GYRO_OFFSET);

        m_swerveMechanism = new SwerveMechanism(
                new ModulesHolder(
                        m_swerveModules[0],
                        m_swerveModules[1],
                        m_swerveModules[2],
                        m_swerveModules[3]
                ),
                new SwerveSpecs(MAX_VELOCITY, MAX_OMEGA_RAD_PER_SEC, MAX_ACC, MAX_FRONT_ACC, MAX_SIDE_ACC, MAX_FORWARD_ACC),
                m_gyro,
                INITIAL_SWERVE_POSITION
        );

        m_angleController = new PIDController(ANGLE_PID_GAINS.kp, ANGLE_PID_GAINS.ki, ANGLE_PID_GAINS.kd);
        m_xController = new PIDController(TRANSLATION_PID_GAINS.kp, TRANSLATION_PID_GAINS.ki, TRANSLATION_PID_GAINS.kd);
        m_yController = new PIDController(TRANSLATION_PID_GAINS.kp, TRANSLATION_PID_GAINS.ki, TRANSLATION_PID_GAINS.kd);

        m_angleSetpoint = Rotation2d::new;
        m_translationSetpoint = Translation2d::new;

        m_finishTrigger = new Trigger(m_xController::atSetpoint).and(m_yController::atSetpoint).and(m_angleController::atSetpoint).debounce(0.1);
    }

    public Command driveCommand(Supplier<Vector2D> velocityMPS,
                                DoubleSupplier omegaRadPerSec,
                                BooleanSupplier fieldOriented) {
        return m_swerveMechanism.driveCommand(velocityMPS, omegaRadPerSec, fieldOriented, true, this);
    }

    /**
     * A method that turns the robot to a desired angle.
     *
     * @param angleSetpoint The desired angle in radians.
     * @return A command that turns the robot to the wanted angle.
     */
    public Command turnToAngleCommand(Supplier<Vector2D> velocityMPS, Supplier<Rotation2d> angleSetpoint) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_angleSetpoint = angleSetpoint),
                driveCommand(
                        velocityMPS,
                        () -> m_angleController.calculate(m_swerveMechanism.getRotation2D().getRadians(), angleSetpoint.get().getRadians()),
                        () -> true
                )
        ).withName("Turn To Angle");
    }

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
                            double distance = m_swerveMechanism.getPose2D().getTranslation().getDistance(poseSetpoint.get().getTranslation());
                            if (AllianceUtils.isRedAlliance()) return vel.rotate(kPi);
                            return vel;
                        },
                        () -> m_angleController.calculate(m_swerveMechanism.getRotation2D().getRadians(), poseSetpoint.get().getRotation().getRadians()),
                        () -> true
                )
        ).until(m_finishTrigger).withName("PID To Pose");
    }

    public Runnable getOdometryUpdaterRunnable() {
        return m_swerveMechanism::updateOdometry;
    }

    @Override
    public void periodic() {
        m_swerveMechanism.periodic();
    }
}
