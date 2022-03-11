// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.DriverAction;
import com.team2930.lib.Limelight;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
  private Limelight limelight; 
  private Drivetrain m_drivetrain;
  private double distance_meters = 0.0;
  private NetworkTable table;
  public double pitch;
  public double latency;
  public double target;
  public double rotation;

  /** Creates a new Limelight. */
  public LimelightSubsystem(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    this.limelight = new Limelight();
  }

  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    pitch = table.getEntry("ty").getDouble(0);
    latency = (table.getEntry("tl").getDouble(0))*1000;
    target = table.getEntry("tv").getDouble(0);
    rotation = table.getEntry("ts").getDouble(0);

    if (limelight.seesTarget()) {
      distance_meters = limelight.getDist(
        Constants.LimelightConstants.TARGET_HEIGHT_INCHES, 
        Constants.LimelightConstants.LIMELIGHT_HEIGHT_INCHES , 
        Constants.LimelightConstants.LIMELIGHT_PITCH_DEGREES);
      SmartDashboard.putNumber("distance ft", Units.metersToFeet(distance_meters*12));
    }
    SmartDashboard.putNumber("pipelineLatency", latency);
  }

  /** Returns the distance in meters */
  public double hubDistanceMeters () {
    return distance_meters;
  }

  /** Returns the rotation degrees */
  public double hubRotationDegrees () {
    return rotation;
  }

  public Pose2d getRobotPose() {
    if(target==1){
      var roboPose = PhotonUtils.estimateFieldToRobot(
      Units.inchesToMeters(Constants.LimelightConstants.LIMELIGHT_HEIGHT_INCHES), 
      Units.inchesToMeters(Constants.LimelightConstants.TARGET_HEIGHT_INCHES), 
      Units.degreesToRadians(Constants.LimelightConstants.LIMELIGHT_PITCH_DEGREES), 
      pitch, 
      Rotation2d.fromDegrees(rotation), 
      m_drivetrain.getGyroscopeRotation(),
      Constants.HubCentricConstants.HUB_CENTER_POSE2D, 
      Constants.LimelightConstants.LIMELIGHT_TO_ROBOT
      );
      return roboPose;
    } else {
      return null;
    }
  }
}
