package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  // Define camera names as they appear in the PhotonVision dashboard
  private final String[] cameraNames = {
    "FrontCam", "BackLeftCam", "BackRightCam"
  }; // One of these cameras may or may not exist
  private final List<PhotonCamera> cameras = new ArrayList<>();
  AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Simulation objects
  private VisionSystemSim visionSim;
  private final List<PhotonCameraSim> cameraSims = new ArrayList<>();

  List<PhotonPoseEstimator> photonEstimators;
  private List<Optional<EstimatedRobotPose>> latestPoses;

  public VisionSubsystem() {
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(tagLayout);
    }

    // Define shared properties for the cameras
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    // Define physical mounting positions (Robot-to-Camera transforms)
    Transform3d[] robotToCamTransforms = { // These values are from the pigeon
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)), // Front (Driver camera)
      new Transform3d(
          new Translation3d(-9.4, -9.4, -6.5),
          new Rotation3d(0, 0, Math.PI)), // Left (This may not exist)
      new Transform3d(
          new Translation3d(10, -9.5, -6.5),
          new Rotation3d(
              0, 25, Math.PI)) // Back right camera (25° up), xyz values rough inch estimates
    };

    photonEstimators = new ArrayList<>();
    latestPoses = new ArrayList<>();
    for (int i = 0; i < cameraNames.length; i++) {
      PhotonCamera cam = new PhotonCamera(cameraNames[i]);
      cameras.add(cam);
      photonEstimators.add(new PhotonPoseEstimator(tagLayout, robotToCamTransforms[i]));
      latestPoses.add(Optional.empty());

      if (RobotBase.isSimulation()) {
        PhotonCameraSim camSim = new PhotonCameraSim(cam, cameraProp);
        visionSim.addCamera(camSim, robotToCamTransforms[i]);
        cameraSims.add(camSim);
      }
    }
  }

  @Override
  public void periodic() {
    // Process results from all cameras
    int idx = 0;
    latestPoses.clear();
    for (PhotonCamera cam : cameras) {
      for (PhotonPipelineResult result : cam.getAllUnreadResults()) {
        if (result.hasTargets() && result.getBestTarget().getFiducialId() > 0) {
          latestPoses.add(photonEstimators.get(idx).estimateCoprocMultiTagPose(result));
        }
      }
      idx++;
    }
  }

  public void updateVisionPoseSim(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      visionSim.update(pose);
    }
  }

  public List<Optional<EstimatedRobotPose>> getLatestPoses() {
    return latestPoses;
  }

  /**
   * Searches all cameras for the closest AprilTag.
   *
   * @return An Optional containing the closest target found by any camera.
   */
  public Optional<PhotonTrackedTarget> getClosestTag() {
    return cameras.stream()
        .map(cam -> cam.getLatestResult())
        .filter(result -> result.hasTargets())
        .flatMap(result -> result.getTargets().stream())
        .filter(t -> t.getFiducialId() > 0)
        .min(Comparator.comparingDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm()));
  }
}
