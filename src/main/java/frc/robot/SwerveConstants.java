package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.imu.Pigeon;

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

    public static final double PID_TOLERANCE = 0;

    public static final double TRACK_WIDTH = 0; // Meters
    public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2);
    public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2);
    public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2);

    public static final double MAX_MODULE_VEL = 0;
    public static final double MAX_FRONT_ACC = 0;
    public static final double MAX_SIDE_ACC = 0;
    public static final double MAX_SKID_ACC = 0;
    public static final double MAX_FORWARD_ACC = 0;
    public static final double MAX_VEL = 0;
    public static final double MAX_OMEGA_RAD_PER_SEC = 0;
    public static final double MAX_OMEGA_RAD_PER_SEC_SQUARE = 0;

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

    public static final CANcoder FRONT_LEFT_ABS_ENCODER = new CANcoder(0, SWERVE_CANBUS);
    public static final CANcoder FRONT_RIGHT_ABS_ENCODER = new CANcoder(0, SWERVE_CANBUS);
    public static final CANcoder BACK_RIGHT_ABS_ENCODER = new CANcoder(0, SWERVE_CANBUS);
    public static final CANcoder BACK_LEFT_ABS_ENCODER = new CANcoder(0, SWERVE_CANBUS);

    public static final double VELOCITY_CONVERSION_FACTOR = 0;
    public static final double POSITION_CONVERSION_FACTOR = 0;
    public static final double ROTATION_VELOCITY_CONVERSION_FACTOR = 0;

    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(0, 0, 0);
    public static final PIDConstants ANGLE_PID_CONSTANTS = new PIDConstants(0, 0, 0);
    public static final Gains ANGLE_GAINS = new Gains(0, 0, 0);
    public static final Gains TRANSLATION_GAINS = new Gains(0, 0, 0);

    public static final Gains SWERVE_DRIVE_MODULE_GAINS = new Gains(0, 0, 0, 0, 0, 0,0);
    public static final Gains SWERVE_ROTATION_MODULE_GAINS = new Gains(0, 0, 0, 0, 0, 0,0);

    public static final IMU GYRO = new Pigeon(GYRO_ID, SWERVE_CANBUS, new Rotation3d());


}

