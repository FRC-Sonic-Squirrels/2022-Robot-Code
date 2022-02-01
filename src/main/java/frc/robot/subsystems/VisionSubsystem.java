package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
  private PhotonCamera m_camera;
  private PhotonPipelineResult m_result;

  PhotonTrackedTarget m_target;
  private double m_yaw;
  double m_pitch;
  double m_area;
  double m_skew;
  Transform2d m_pose;
  List<TargetCorner> m_corners;

  
  public VisionSubsystem(){
   m_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  
  }

  public PhotonTrackedTarget getTarget(){
    return m_target;
  }

  public double getYaw(){
    return m_yaw;
  }

  @Override
  public void periodic() {
    m_result = m_camera.getLatestResult();

    m_target = m_result.getBestTarget();
    m_yaw = m_target.getYaw();
    m_pitch = m_target.getPitch();
    m_area = m_target.getArea();
    m_skew = m_target.getSkew();
    m_pose = m_target.getCameraToTarget();
    m_corners = m_target.getCorners();
  }
}
