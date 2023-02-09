package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AprilTagSubsystem extends SubsystemBase {

    // Define the things that the subsystem needs to use
    public Drivetrain drivetrain;
    public PhotonCamera photonCamera = new PhotonCamera("photonvision");

    public AprilTagSubsystem(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    // Each cycle of the robot loop, this method is called
    public void periodic() {
        // Get the latest result from the camera
        var result = photonCamera.getLatestResult();

        // If the camera sees a target
        if (result.hasTargets()) {
            // Gather the data from the target
            var target = result.getBestTarget();
            var targetYaw = target.getYaw();
            var targetPitch = target.getPitch();
            var targetDistance = target.getBestCameraToTarget();
            var targetSkew = target.getSkew();
            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();

            // Get the AprilTag object from the constants file
            AprilTag tag = Constants.AprilTagPositions.TAGS.get(targetID);

            // Calculate the distance to the target
            double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.Camera.CAMERA_HEIGHT, tag.pose.getZ(), Constants.Camera.CAMERA_PITCH, targetPitch);

            // TODO: Use the data to drive the robot to the target            
        }
    }

}