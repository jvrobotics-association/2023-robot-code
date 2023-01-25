package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AprilTagSubsystem extends SubsystemBase {

    public Drivetrain drivetrain;
    public PhotonCamera photonCamera = new PhotonCamera("photonvision");

    public AprilTagSubsystem(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void periodic() {

        var result = photonCamera.getLatestResult();

        if (result.hasTargets()) {
            var target = result.getBestTarget();
            var targetYaw = target.getYaw();
            var targetPitch = target.getPitch();
            var targetDistance = target.getBestCameraToTarget();
            var targetSkew = target.getSkew();
            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();

            AprilTag tag = Constants.AprilTagPositions.TAGS.get(targetID);


            double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.Camera.CAMERA_HEIGHT, tag.pose.getZ(), Constants.Camera.CAMERA_PITCH, targetPitch);
        }
    }

}