package frc.robot.subsystems.swerve_example;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.math.Circle;
import frc.excalib.swerve.swerve_utils.SwerveModuleType;

import static frc.excalib.swerve.swerve_utils.SwerveModuleType.MK4i_l3;

public class Constants {
    public static final String SWERVE_CANBUS = "CTRESwerve";

    // Motors & Sensors IDs
    public static final int FRONT_LEFT_DRIVE_ID = 0;
    public static final int FRONT_RIGHT_DRIVE_ID = 0;
    public static final int BACK_RIGHT_DRIVE_ID = 0;
    public static final int BACK_LEFT_DRIVE_ID = 0;

    public static final int FRONT_LEFT_ROTATION_ID = 0;
    public static final int FRONT_RIGHT_ROTATION_ID = 0;
    public static final int BACK_RIGHT_ROTATION_ID = 0;
    public static final int BACK_LEFT_ROTATION_ID = 0;

    public static final int FRONT_LEFT_CANCODER_ID = 0;
    public static final int FRONT_RIGHT_CANCODER_ID = 0;
    public static final int BACK_RIGHT_CANCODER_ID = 0;
    public static final int BACK_LEFT_CANCODER_ID = 0;

    public static final int GYRO_ID = 0;
    public static final Rotation3d GYRO_OFFSET = new Rotation3d();

    // Module Specs
    public static final SwerveModuleType MODULE_TYPE = MK4i_l3;

    // Driving Mechanism Constraints
    public static final double MAX_MODULE_VELOCITY = 0;
    public static final double MAX_MODULE_ACCELERATION = 0;
    public static final double MAX_MODULE_JERK = 0;
    public static final int STALL_DRIVING_MODULE_LIMIT = 0;
    public static final int FREE_DRIVING_MODULE_LIMIT = 0;
    public static final Gains DRIVING_MODULE_GAINS = new Gains();

    // Steering Mechanism Constraints
    public static final int STALL_STEERING_MODULE_LIMIT = 0;
    public static final int FREE_STEERING_MODULE_LIMIT = 0;
    public static final Gains STEERING_MODULE_GAINS = new Gains();
    public static final double STEERING_PID_TOLERANCE = 0;

    public static final Translation2d[] MODULE_LOCATIONS =
            new Translation2d[]{
                    new Translation2d(),
                    new Translation2d(),
                    new Translation2d(),
                    new Translation2d()
            };

    // Swerve Constraints
    public static final double MAX_VELOCITY = 0;
    public static final double MAX_OMEGA_RAD_PER_SEC = 0;
    public static final double MAX_ACC = 0;
    public static final double MAX_FRONT_ACC = 0;
    public static final double MAX_SIDE_ACC = 0;
    public static final double MAX_FORWARD_ACC = 0;

    public static final Pose2d INITIAL_SWERVE_POSITION = new Pose2d();

    // Auto Drive Constants
    public static final Gains ANGLE_PID_GAINS = new Gains();
    public static final Gains TRANSLATION_PID_GAINS = new Gains();

    public static final PIDConstants ANGLE_PID_PP_CONSTANTS = new PIDConstants(0.0, 0.0, 0.0);
    public static final PIDConstants TRANSLATION_PID_PP_CONSTANTS = new PIDConstants(0.0, 0.0, 0.0);

    // Obstacle Avoidance Constants
    public static final Circle OBSTACLE = new Circle(0, 0, 1);
    public static final double DISTANCE_TO_AVOID_OBSTACLE = 2; // m
    public static final double POSSIBLE_DEVIATION = Math.PI / 18;
}
