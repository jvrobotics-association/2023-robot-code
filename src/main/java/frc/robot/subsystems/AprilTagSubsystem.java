package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AprilTagSubsystem extends SubsystemBase {

    // Define the things that the subsystem needs to use
    public Swerve swerve;
    public PhotonCamera photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    public AprilTagSubsystem(Swerve _swerve) {
        this.swerve = _swerve;
        // chose the april tag pipeline
        photonCamera.setPipelineIndex(0);
    }

    public Pose3d calculateRobotPosition() {
        // Get the latest result from the camera
        var result = photonCamera.getLatestResult();

        SmartDashboard.putBoolean("Targeting AprilTag", result.hasTargets());

        // If the camera sees a target
        if (result.hasTargets()) {
            // Gather the data from the target
            var target = result.getBestTarget();
            var targetYaw = target.getYaw();
            var targetPitch = target.getPitch();
            var targetTransform = target.getBestCameraToTarget();
            var targetSkew = target.getSkew();
            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();

            // Get the AprilTag object from the constants file
            AprilTag tag = Constants.AprilTagPositions.tags.get(targetID);

            // If the tag is null, then we don't know where it is
            if (tag == null) {
                return null;
            }

            // Create a transform for the camera
            Transform3d cameraPose = new Transform3d(
                    new Translation3d(Constants.Camera.cameraXOffset, Constants.Camera.cameraYOffset,
                            Constants.Camera.cameraHeight),
                    new Rotation3d(Constants.Camera.cameraPitch, 0, 0));

            // Calculate the robot's position on the field
            Pose3d robotOnFieldPose = PhotonUtils.estimateFieldToRobotAprilTag(targetTransform.inverse(), tag.pose,
                    cameraPose);

            // Print the robot's position to the SmartDashboard
            SmartDashboard.putString("Estimated Robot Position", robotOnFieldPose.toString());

            return robotOnFieldPose;

        }

        return null;
    }

    public void updateRobotPosition() {
        Pose3d robotPose = calculateRobotPosition();

        if (robotPose != null) {
            swerve.resetOdometry(robotPose.toPose2d());
        }
    }

}
