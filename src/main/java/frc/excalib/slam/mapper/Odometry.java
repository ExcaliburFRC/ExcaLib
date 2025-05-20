package frc.excalib.slam.mapper;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.Supplier;

/**
 * A wrapper class for the WPILib odometry
 */
public class Odometry extends SwerveDrivePoseEstimator {
    private final Supplier<Rotation2d> m_YAW_SUPPLIER;
    private Pose2d m_robotPose;

    /**
     * @param swerveDriveKinematics the kinematics of the swerve
     * @param modulesPositions an array of the module positions
     * @param angleSupplier a DoubleSupplier represents the angle of the robot
     * @param initialPose the initial position of the robot
     */
    public Odometry(
            SwerveDriveKinematics swerveDriveKinematics,
            SwerveModulePosition[] modulesPositions,
            Supplier<Rotation2d> angleSupplier,
            Pose2d initialPose) {
        super(swerveDriveKinematics, angleSupplier.get(), modulesPositions, initialPose);
        m_YAW_SUPPLIER = angleSupplier;
        m_robotPose = initialPose;
    }

    /**
     * @return the current estimated robot position
     */
    public Pose2d getRobotPose() {
        return m_robotPose;
    }

    /**
     * updates the odometry. should be called periodically
     *
     * @param modulesPositions an array of the module positions
     */
    public void updateOdometry(SwerveModulePosition[] modulesPositions) {
        m_robotPose = super.update(
                m_YAW_SUPPLIER.get(),
                modulesPositions
        );
    }

    /**
     * resets the odometry to a wanted position
     *
     * @param modulesPositions an array of the module positions
     * @param newInitialPose the new initial position of the robot
     */
    public void resetOdometry(SwerveModulePosition[] modulesPositions, Pose2d newInitialPose) {
        super.resetPosition(m_YAW_SUPPLIER.get(), modulesPositions, newInitialPose);
    }
}
