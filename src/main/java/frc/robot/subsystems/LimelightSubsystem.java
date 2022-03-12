// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.team2930.lib.Limelight;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  private static double rateMetersPerSecond = 1.0;
  private static final SlewRateLimiter distanceRateLimiter = new SlewRateLimiter(rateMetersPerSecond, 0.0);

  /** Creates a new Limelight. */
  public LimelightSubsystem(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    this.limelight = new Limelight();
  }

  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    pitch = table.getEntry("ty").getDouble(0);
    latency = (table.getEntry("tl").getDouble(0));
    target = table.getEntry("tv").getDouble(0);
    rotation = table.getEntry("ts").getDouble(0);

    if (limelight.seesTarget()) {
      distance_meters = distanceRateLimiter.calculate(
          limelight.getDist(Units.inchesToMeters(Constants.LimelightConstants.TARGET_HEIGHT_INCHES),
              Units.inchesToMeters(Constants.LimelightConstants.LIMELIGHT_HEIGHT_INCHES),
              Units.inchesToMeters(Constants.LimelightConstants.LIMELIGHT_PITCH_DEGREES)));
      SmartDashboard.putNumber("distance ft", Units.metersToFeet(distance_meters));
    }
    else {
      // return zero if we don't see the target
      SmartDashboard.putNumber("distance ft", 0);
    }
    SmartDashboard.putNumber("pipelineLatency", latency);
  }

  public boolean seesTarget() {
    return limelight.seesTarget();
  }

  /** Returns the distance in meters */
  public double getDistanceMeters () {
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
      m_drivetrain.getPose().getRotation(),  // TODO: double check this, was getGyroAngle()
      new Pose2d(
        Constants.LimelightConstants.HIGH_HUB_RADIUS_INCHES * Math.cos(-(rotation+m_drivetrain.getPose().getRotation().getRadians())), 
        Constants.LimelightConstants.HIGH_HUB_RADIUS_INCHES * Math.sin(-(rotation+m_drivetrain.getPose().getRotation().getRadians())), 
        new Rotation2d(Units.degreesToRadians(rotation))),
      Constants.LimelightConstants.LIMELIGHT_TO_ROBOT
      );
      return roboPose;
    } else {
      return null;
    }
  }
}
