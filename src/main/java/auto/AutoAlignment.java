package auto;

import calc.PhotonCameraPose;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import hardware.Swerve;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

public class AutoAlignment {

    PhotonCameraPose photonCameraPose;
    Swerve swerve;

    // as of Charged Up 2023, there is no tag with ID 0
    private int tagID = 0;

    private Pose2d targetPose;

    public AutoAlignment(Swerve swerve) {
        this.swerve = swerve;
        photonCameraPose = new PhotonCameraPose();
    }

    public void calibrateOdometry() {
        // get the estimated robot pose from the photon camera
        Optional<EstimatedRobotPose> result = photonCameraPose.getEstimatedRobotPose(swerve.getPose());

        // if the estimated robot pose is present, set the swerve odometry to the estimated robot pose
        if (result.isPresent()) {

            EstimatedRobotPose estimatedRobotPose = result.get();

            // add the vision measurement to the pose estimator and update the odometry
            swerve.getPoseEstimator().addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), Timer.getFPGATimestamp());

        }

        if (photonCameraPose.getTagLayout().getTagPose(tagID).isPresent()) {
            // get the tag pose
            setTargetPose(photonCameraPose.getTagLayout().getTagPose(tagID).get().toPose2d());
        }
    }

    public void moveToTargetPose(){
        calibrateOdometry();

        ChassisSpeeds alignmentSpeeds = SwerveTrajectory.HDC.calculate(
                swerve.getPose(),
                new Pose2d(
                        swerve.getPose().getX(),
                        getTargetPose().getY(),
                        getTargetPose().getRotation()),
                        // Notice the 0 m/s here
                        // This is because we want the robot to end at a stop,
                        0,
                        getTargetPose().getRotation()
                );

        ChassisSpeeds controllerSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                swerve.getDriver().getLeftX(),
                0,
                0,
                swerve.getPose().getRotation()
        );

        ChassisSpeeds combinedSpeeds = new ChassisSpeeds (
                controllerSpeeds.vxMetersPerSecond + alignmentSpeeds.vxMetersPerSecond,
                controllerSpeeds.vyMetersPerSecond + alignmentSpeeds.vyMetersPerSecond,
                controllerSpeeds.omegaRadiansPerSecond + alignmentSpeeds.omegaRadiansPerSecond
        );

        swerve.drive(
                combinedSpeeds.vxMetersPerSecond,
                combinedSpeeds.vyMetersPerSecond,
                combinedSpeeds.omegaRadiansPerSecond,
                false,
                false);
    }

    public void setTagID(int tagID) {
        this.tagID = tagID;
    }

    public int getTagID() {
        return tagID;
    }

    public Pose2d getTargetPose(){
        return this.targetPose;
    }

    public void setTargetPose(Pose2d targetPose){
        this.targetPose = targetPose;
    }
}
