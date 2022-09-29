package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase{
  private PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private Drivetrain m_drivetrain;
  private double latencySeconds;
  private PhotonPipelineResult result;
  private VisionPipeline mode;

  private double target, pitch, rotation;

  // enum might be unnecessary, still unsure
  public enum VisionPipeline {
    RED,
    BLUE
  }

  
  //if you want to get pitch, yaw etc. call the getResult method. This will return the latest result 
  //you can check if the result has targets result.hasTargets() 
  //if it does you can do result.getBestTarget()
  //you can now access pitch,yaw etc from that target object 
  //this is to prevent null errors and pipeline d-sync 


  public VisionSubsystem(Drivetrain drivetrain){
   m_drivetrain = drivetrain;
   m_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
   //how do we know which index is which i.e red pipeline/blue pipeline
   // index 1 is red, index 2 is blue
   m_camera.setPipelineIndex(0);
  }

  public PhotonPipelineResult getResult(){
    return m_result;
  }
  
  @Override
  public void periodic() {
    m_result = m_camera.getLatestResult();
    if(m_result.hasTargets()){
      SmartDashboard.putNumber("yaw", m_result.getBestTarget().getYaw());
    } else {
      SmartDashboard.putNumber("yaw", -200);
    }
    SmartDashboard.putBoolean("has targets", m_result.hasTargets());
    
  }

  public Pose2d getRobotPose() {
    if(target==1){
      Pose2d roboPose = PhotonUtils.estimateFieldToRobot(
      Units.inchesToMeters(Constants.VisionConstants.CAMERA_HEIGHT_INCHES), 
      Units.inchesToMeters(Constants.VisionConstants.TARGET_HEIGHT_INCHES), 
      Units.degreesToRadians(Constants.VisionConstants.CAMERA_PITCH_DEGREES), 
      pitch, 
      Rotation2d.fromDegrees(rotation), 
      m_drivetrain.getRotation(),
      Constants.HubCentricConstants.HUB_CENTER_POSE2D, 
      Constants.VisionConstants.CAMERA_TO_ROBOT
      );
      return roboPose;
    } else {
      return null;
    }
  }

  public void setPipelineRed() {
    m_camera.setPipelineIndex(1);
  }

  public void setPipelineBlue() {
    m_camera.setPipelineIndex(2);
  }
  
}
