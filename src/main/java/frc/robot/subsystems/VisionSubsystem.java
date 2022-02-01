package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
   //how do we know which index is which i.e red pipeline/blue pipeline 
   m_camera.setPipelineIndex(0);
  }

  public PhotonTrackedTarget getTarget(){
    return m_target;
  }

  public double getYaw(){
    return m_yaw;
  }

  public Transform2d getPoseToCargo(){
    return m_pose;
  }
  @Override
  public void periodic() {
    m_result = m_camera.getLatestResult();

    if(m_result.hasTargets()){
      m_target = m_result.getBestTarget();

      m_yaw = m_target.getYaw();
      m_pitch = m_target.getPitch();
      m_area = m_target.getArea();
      m_skew = m_target.getSkew();
      m_pose = m_target.getCameraToTarget();
      m_corners = m_target.getCorners();
    } else {
      m_target = null;
      m_yaw = 0.0;
      m_pitch = 0.0;
      m_area = 0.0;
      m_skew = 0.0;
      m_pose = null;
      m_corners = null;
    }
  }
}
