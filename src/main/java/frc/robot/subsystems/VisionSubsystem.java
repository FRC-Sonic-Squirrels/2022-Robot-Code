package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
  PhotonCamera m_intakeCamera;
  PhotonPipelineResult m_intakeResult;

  PhotonTrackedTarget m_intakeTarget;
  double m_intakeYaw;
  double m_intakePitch;
  double m_intakeArea;
  double m_intakeSkew;
  Transform2d m_intakePose;
  List<TargetCorner> m_intakeCorners;

  PhotonCamera m_rearCamera;
  PhotonPipelineResult m_rearResult;

  PhotonTrackedTarget m_rearTarget;
  double m_rearYaw;
  double m_rearPitch;
  double m_rearArea;
  double m_rearSkew;
  Transform2d m_rearPose;
  List<TargetCorner> m_rearCorners;
  double m_actualRearYaw;

  public VisionSubsystem(){
   m_intakeCamera = new PhotonCamera("IntakeCam");
   m_rearCamera = new PhotonCamera("IntakeCam");
  
  }

  @Override
  public void periodic() {
    m_intakeResult = m_intakeCamera.getLatestResult();

    m_intakeTarget = m_intakeResult.getBestTarget();
    m_intakeYaw = m_intakeTarget.getYaw();
    m_intakePitch = m_intakeTarget.getPitch();
    m_intakeArea = m_intakeTarget.getArea();
    m_intakeSkew = m_intakeTarget.getSkew();
    m_intakePose = m_intakeTarget.getCameraToTarget();
    m_intakeCorners = m_intakeTarget.getCorners();

    m_rearResult = m_intakeCamera.getLatestResult();

    m_rearTarget = m_rearResult.getBestTarget();
    m_rearYaw = m_rearTarget.getYaw();
    m_rearPitch = m_rearTarget.getPitch();
    m_rearArea = m_rearTarget.getArea();
    m_rearSkew = m_rearTarget.getSkew();
    m_rearPose = m_rearTarget.getCameraToTarget();
    m_rearCorners = m_rearTarget.getCorners();

    if(m_rearYaw>=0){
      m_actualRearYaw=m_rearYaw-180;
    } else{
      m_actualRearYaw=m_rearYaw+180;
    };
  }
}
