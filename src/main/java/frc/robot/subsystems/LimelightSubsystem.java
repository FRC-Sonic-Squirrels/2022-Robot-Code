// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.team2930.lib.Limelight;
import com.team2930.lib.ll_mode;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
  private Limelight limelight; 
  private Drivetrain m_drivetrain;
  private double distance_meters = 0.0;
  private NetworkTable table;
  private double pitch;
  private double yaw;
  private double latency;
  private boolean seesTargetRecently = false;
  private boolean seesTarget = false;
  private double targetHeadingDegrees;
  private Pose2d robotPose = new Pose2d();
  private boolean ledsOn = true;
  private Pose2d kalmanLimelightPose;
  private double kalmanHubDistFeet;

  private MedianFilter seesTargetFilter = new MedianFilter(5);
  // TODO: add linear filter to limelight distance 

  /** Creates a new Limelight. */
  public LimelightSubsystem(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    this.limelight = new Limelight("limelight-one");
    limelight.setPipeline(0);
  }

  @Override
  public void periodic() {

    table = NetworkTableInstance.getDefault().getTable("limelight-one");
    pitch = table.getEntry("ty").getDouble(0);
    yaw = table.getEntry("tx").getDouble(0);
    latency = table.getEntry("tl").getDouble(0);
    seesTargetRecently = seesTargetFilter.calculate(table.getEntry("tv").getDouble(0)) > 0.9;
    seesTarget = table.getEntry("tv").getDouble(0) > 0.99;

    robotPose = m_drivetrain.getPose();

    double currentHeadingDegrees = robotPose.getRotation().getDegrees();
    if (seesTarget) {
      // yaw is reported negative in Counter Clockwise direction, need to reverse it.
      targetHeadingDegrees = (currentHeadingDegrees - yaw);

      distance_meters = limelight.getDist(
        Units.inchesToMeters(Constants.LimelightConstants.HIGH_HUB_HEIGHT_INCHES),
        Units.inchesToMeters(Constants.LimelightConstants.LIMELIGHT_HEIGHT_INCHES),
        Constants.LimelightConstants.LIMELIGHT_PITCH_DEGREES) 
        + Units.inchesToMeters(Constants.LimelightConstants.HIGH_HUB_RADIUS_INCHES);
    } else if (seesTargetRecently) {
      // we didn't see the target this loop, but on average we have seen it very recently
      // assume distance and heading haven't changed much
    } else {
      // return zero if we don't see the target
      distance_meters = 0;
      // no target, so hold current rotation
      targetHeadingDegrees = currentHeadingDegrees;
    }

    // SmartDashboard.putBoolean("LL has target", seesTarget);
    // SmartDashboard.putNumber("LL pitch", pitch);
    SmartDashboard.putBoolean("LL seen target recently", seesTargetRecently);
    SmartDashboard.putNumber("LL distance inches", Units.metersToInches(distance_meters));
    // SmartDashboard.putNumber("LL distance ft", Units.metersToFeet(distance_meters));
    // SmartDashboard.putNumber("LL target heading deg", targetHeadingDegrees);
    // SmartDashboard.putNumber("LL pipelineLatency", latency);

    
  }

  public boolean onTarget() {
    if ((seesTarget) && 
        Math.abs(yaw) < Constants.LimelightConstants.TARGET_TOLERANCE_DEGREES) {
      return true;
    } else {
      return false;
    }
  }

  public Pose2d getLimelightPoseMeters() {
    return kalmanLimelightPose;
  }

  public double getKalmanHubDistanceFeet(){
    return kalmanHubDistFeet;
  }

  public boolean seesTarget() {
    return seesTarget;
  }

  public boolean seesTargetRecently() {
    return seesTargetRecently;
  }

  /** 
   * Returns the distance in meters 
   */
  public double getDistanceMeters () {
    return distance_meters;
  }

  /** 
   * Returns the heading of the vision target in Radians.
   */
  public double getTargetHeadingRadians() {
    return Units.degreesToRadians(targetHeadingDegrees);
  }

  /** 
   * the angle offset to the vision target
   */
  public double targetYaw () {
    return yaw;
  }

  /**
   * Toggle the LimeLight LEDs on or off
   */
  public void toggleLEDs() {
    ledsOn = !ledsOn;
    if (ledsOn) {
      limelight.setLEDMode(ll_mode.led.on);
    } else {
      limelight.setLEDMode(ll_mode.led.off);
    }
  }

  /*
  * Returns an estimated pose of the robot based on the limelight sighting of the target.
  *
  * WARNING: This is UNTESTED.
  */
  public Pose2d getEstimatedRobotPose() {
    if (seesTarget) {
      Pose2d estimatedRobotPose = PhotonUtils.estimateFieldToRobot(
      Units.inchesToMeters(Constants.LimelightConstants.LIMELIGHT_HEIGHT_INCHES), 
      Units.inchesToMeters(Constants.LimelightConstants.HIGH_HUB_HEIGHT_INCHES), 
      Units.degreesToRadians(Constants.LimelightConstants.LIMELIGHT_PITCH_DEGREES), 
      Units.degreesToRadians(pitch),
      new Rotation2d(Units.degreesToRadians(-yaw)), 
      robotPose.getRotation(),
      new Pose2d(
        Constants.LimelightConstants.HIGH_HUB_RADIUS_INCHES * Math.cos(-(yaw+robotPose.getRotation().getRadians())), 
        Constants.LimelightConstants.HIGH_HUB_RADIUS_INCHES * Math.sin(-(yaw+robotPose.getRotation().getRadians())), 
        new Rotation2d(Units.degreesToRadians(yaw))),
      Constants.LimelightConstants.LIMELIGHT_TO_ROBOT
      );
      return estimatedRobotPose;
    } else {
      return null;
    }
  }

  public double getPoseDistanceToHub(){
    return Math.sqrt(
        Math.pow(Constants.HubCentricConstants.HUB_CENTER_POSE2D.getX() - m_drivetrain.getPose().getX(), 2) + 
        Math.pow(Constants.HubCentricConstants.HUB_CENTER_POSE2D.getY() - m_drivetrain.getPose().getY(), 2));
  }
}
