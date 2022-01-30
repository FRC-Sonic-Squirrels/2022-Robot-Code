package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Transform2d;

public class VisionSubsystem {
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  PhotonPipelineResult result = camera.getLatestResult();

  PhotonTrackedTarget target = result.getBestTarget();
  double yaw = target.getYaw();
  double pitch = target.getPitch();
  double area = target.getArea();
  double skew = target.getSkew();
  Transform2d pose = target.getCameraToTarget();
  List<TargetCorner> corners = target.getCorners();
}
