package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.imu.Pigeon;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.swerve.ModulesHolder;
import frc.excalib.swerve.Swerve;
import frc.excalib.swerve.SwerveModule;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;

public class SwerveConstants {
    public static final int FRONT_LEFT_DRIVE_ID = 0;
    public static final int FRONT_RIGHT_DRIVE_ID = 0;
    public static final int BACK_RIGHT_DRIVE_ID = 0;
    public static final int BACK_LEFT_DRIVE_ID = 0;

    public static final int FRONT_LEFT_ROTATION_ID = 0;
    public static final int FRONT_RIGHT_ROTATION_ID = 0;
    public static final int BACK_RIGHT_ROTATION_ID = 0;
    public static final int BACK_LEFT_ROTATION_ID = 0;

    public static final int GYRO_ID = 0;
    public static final String SWERVE_CANBUS = "Swerve";

    private static final double PID_TOLERANCE = 0;

    public static final double TRACK_WIDTH = 0; // Meters
    public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2);
    public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2);
    public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2);

    public static final double MAX_MODULE_VEL = 4.45;
    public static final double MAX_FRONT_ACC = 2;
    public static final double MAX_SIDE_ACC = 6;
    public static final double MAX_SKID_ACC = 9;
    public static final double MAX_FORWARD_ACC = 9;
    public static final double MAX_VEL = 4.5;
    public static final double MAX_OMEGA_RAD_PER_SEC = 4;
    public static final double MAX_OMEGA_RAD_PER_SEC_SQUARE = 1;

    public static final double ANGLE_CONTROLLER_TOLORANCE = 0;
    public static final double X_CONTROLLER_TOLORANCE = 0;
    public static final double Y_CONTROLLER_TOLORANCE = 0;

    public static final PathConstraints MAX_PATH_CONSTRAINTS = new PathConstraints(
            MAX_VEL,
            MAX_SKID_ACC,
            MAX_OMEGA_RAD_PER_SEC,
            MAX_OMEGA_RAD_PER_SEC_SQUARE,
            12.0,
            false
    );

    private static final CANcoder FRONT_LEFT_ABS_ENCODER = new CANcoder(0, SWERVE_CANBUS);
    public static final CANcoder FRONT_RIGHT_ABS_ENCODER = new CANcoder(0, SWERVE_CANBUS);
    private static final CANcoder BACK_RIGHT_ABS_ENCODER = new CANcoder(0, SWERVE_CANBUS);
    private static final CANcoder BACK_LEFT_ABS_ENCODER = new CANcoder(0, SWERVE_CANBUS);

    private static final double VELOCITY_CONVERSION_FACTOR = 0;
    private static final double POSITION_CONVERSION_FACTOR = 0;
    private static final double ROTATION_VELOCITY_CONVERSION_FACTOR = 0;

    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(0.0, 0.0, 0.0);
    public static final PIDConstants ANGLE_PID_CONSTANTS = new PIDConstants(0.0, 0.0, 0.0);
    public static final Gains ANGLE_GAINS = new Gains(0.0, 0.0, 0.0);
    public static final Gains TRANSLATION_GAINS = new Gains(0.0, 0.0, 0.0);

    private static final IMU GYRO = new Pigeon(GYRO_ID, SWERVE_CANBUS, new Rotation3d());

    public static Swerve configureSwerve(Pose2d initialPose) {
        return new Swerve(
                new ModulesHolder(
                        new SwerveModule(
                                new TalonFXMotor(FRONT_LEFT_DRIVE_ID, SWERVE_CANBUS),
                                new SparkMaxMotor(FRONT_LEFT_ROTATION_ID, kBrushless),
                                new Gains(0, 0, 0, 0, 0, 0, 0),
                                new Gains(0, 0, 0, 0, 0, 0, 0),
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
                                new Gains(0, 0, 0, 0, 0, 0, 0),
                                new Gains(0, 0, 0, 0, 0, 0, 0),
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
                                new Gains(0, 0, 0, 0, 0, 0, 0),
                                new Gains(0, 0, 0, 0, 0, 0, 0),
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
                                new Gains(0, 0, 0, 0, 0, 0, 0),
                                new Gains(0, 0, 0, 0, 0, 0, 0),
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
}

