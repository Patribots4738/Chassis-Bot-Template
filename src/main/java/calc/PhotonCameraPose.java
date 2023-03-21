package calc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class PhotonCameraPose {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    public PhotonCameraPose() {
        // Load the AprilTag field layout for the 2023 game 
        AprilTagFieldLayout aprilTagFieldLayout = getTagLayout();

        // Load the PhotonCamera
        photonCamera = getPhotonCamera();

        // Create the PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, // This is the default strategy
                photonCamera,
                Constants.VisionConstants.CAMERA_POSITION);
    }

    public AprilTagFieldLayout getTagLayout() {
        AprilTagFieldLayout tagFieldLayout = null;
        try {
            tagFieldLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            System.out.println("April tag field layout not found!");
        }
        return tagFieldLayout;
    }

    private PhotonCamera getPhotonCamera() {
        PhotonCamera camera = null;
        try {
            camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
        } catch (Exception e) {
            System.out.println("Camera not found!");
        }
        return camera;
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d pevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(pevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}
