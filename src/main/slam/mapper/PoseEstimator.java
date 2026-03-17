package frc.excalib.slam.mapper;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.Supplier;

public class PoseEstimator extends SwerveDrivePoseEstimator {
    private final Supplier<Rotation2d> m_YAW_SUPPLIER;
    private Pose2d m_robotPose;

    public PoseEstimator(SwerveDriveKinematics kinematics, Supplier<Rotation2d> yawSupplier, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        super(kinematics, yawSupplier.get(), modulePositions, initialPose);
        m_YAW_SUPPLIER = yawSupplier;
    }


}
