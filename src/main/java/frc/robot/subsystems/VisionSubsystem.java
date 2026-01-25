package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Comparator;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("HubOrientedCameraMountedOnTurret");
  VisionSystemSim visionSim = new VisionSystemSim("main");
  AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera Placement

  Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
  // and pitched 15 degrees up.
  Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
  Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

  PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(tagLayout, robotToCamera);

  SimCameraProperties cameraProp = new SimCameraProperties();
  PhotonCameraSim cameraSim;

  private Pose3d latestPose3d = new Pose3d();
  private Pose2d latestPose2d = new Pose2d();

  public VisionSubsystem() {
    if (RobotBase.isSimulation()) {
      // A 640 x 480 camera with a 100 degree diagonal FOV.
      cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
      // Approximate detection noise with average and standard deviation error in
      // pixels.
      cameraProp.setCalibError(0.25, 0.08);
      // Set the camera image capture framerate (Note: this is limited by robot loop
      // rate).
      cameraProp.setFPS(20);
      // The average and standard deviation in milliseconds of image data latency.
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      cameraSim = new PhotonCameraSim(camera, cameraProp);

      // Add this camera to the vision system simulation with the given
      // robot-to-camera transform.
      visionSim.addCamera(cameraSim, robotToCamera);
      visionSim.addAprilTags(tagLayout);
    }
  }

  @Override
  public void periodic() {
    camera
        .getAllUnreadResults()
        .forEach(
            (result) -> {
              if (result.hasTargets() && result.getBestTarget().getFiducialId() > 0) {
                Optional<EstimatedRobotPose> visionEst =
                    photonEstimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                  visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
                }
                if (visionEst.isPresent()) {
                  latestPose3d = visionEst.get().estimatedPose;
                  latestPose2d = latestPose3d.toPose2d();
                }
              }
            });
  }

  public void updateVisionPoseSim(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      visionSim.update(pose);
    }
  }

  public Pose2d getEstimatedGlobalPose2d() {
    return latestPose2d;
  }

  public Pose3d getEstimatedGlobalPose3d() {
    return latestPose3d;
  }

  public Optional<PhotonTrackedTarget> getClosestTag() {
    return camera.getLatestResult().hasTargets()
        ? camera.getLatestResult().getTargets().stream()
            .filter(t -> t.getFiducialId() > 0) // AprilTags only
            .min(
                Comparator.comparingDouble(
                    t -> t.getBestCameraToTarget().getTranslation().getNorm()))
        : Optional.empty();
  }
}
