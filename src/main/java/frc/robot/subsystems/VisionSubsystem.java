package frc.robot.subsystems;

import java.util.List;
import org.opencv.photo.MergeRobertson;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase{
  private PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private Drivetrain m_drivetrain;

  //if you want to get pitch, yaw etc. call the getResult method. This will return the lastest result 
  //you can check if the result has targets result.hasTargets() 
  //if it does you can do result.getBestTarget()
  //you can now access pitch,yaw etc from that target object 
  //this is to prevent null errors and pipeline d-sync 
  public VisionSubsystem(Drivetrain drivetrain){
   m_drivetrain = drivetrain;
   m_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
   //how do we know which index is which i.e red pipeline/blue pipeline 
   // TODO: add Constants that denote RedCargo and BlueCargo pipelines
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
    PhotonCamera frontCam = new PhotonCamera("limelight"); //TODO: name camera
    var result = frontCam.getLatestResult();
    PhotonTrackedTarget tape = result.getBestTarget();
    double targetPitch = tape.getPitch();
    var roboPose = PhotonUtils.estimateFieldToRobot(
     Units.inchesToMeters(Constants.VisionConstants.cameraHeightInches), 
     Units.inchesToMeters(Constants.VisionConstants.targetHeightInches), 
     Units.degreesToRadians(Constants.VisionConstants.cameraPitchDegrees), 
     targetPitch, 
     Rotation2d.fromDegrees(-tape.getYaw()), 
     m_drivetrain.getGyroscopeRotation(),
     Constants.HubCentricConstants.HUB_CENTER_POSE2D, 
     Constants.VisionConstants.cameraToRobot
     );

    return roboPose;
   }
  
}
