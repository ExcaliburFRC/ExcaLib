package frc.excalib.slam.mapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

public class PhotonAprilTagsCamera implements Logged {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator photonPoseEstimator;

    public PhotonAprilTagsCamera(String cameraName, AprilTagFields aprilTagField, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        camera.setDriverMode(false);

        fieldLayout = AprilTagFieldLayout.loadField(aprilTagField);

        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(LOWEST_AMBIGUITY);
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public void setDriverMode(boolean isDriverMode) {
        camera.setDriverMode(isDriverMode);
    }

    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    public PhotonPipelineResult getFirstLatestResult() {
        var result = camera.getAllUnreadResults();

        if (result != null) return result.getFirst();
        return new PhotonPipelineResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : unreadResults)
            if (result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < 0.2) {
                Translation2d targetTranslation = result.getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d();
                // m
                double TOO_FAR = 3.5;
                if (targetTranslation.getDistance(new Translation2d(0, 0)) < TOO_FAR) {
                    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
                    return photonPoseEstimator.update(result);
                }
            }

        return Optional.empty();
    }

    public boolean updateFromAprilTagPose(BiConsumer<Pose2d, Double> toUpdate) {
        var unreadResults = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : unreadResults)
            if (!result.hasTargets()) return false;

        for (PhotonPipelineResult result : unreadResults) {
            var id = result.getBestTarget().getFiducialId();
            if (id == -1) return false;
            var tag = fieldLayout.getTagPose(id);
            if (tag.isEmpty()) return false;
            toUpdate.accept(
                    tag.get().plus(
                            result.getBestTarget().getBestCameraToTarget()).toPose2d(),
                            result.getTimestampSeconds()
            );
        }
        return true;
    }
}

