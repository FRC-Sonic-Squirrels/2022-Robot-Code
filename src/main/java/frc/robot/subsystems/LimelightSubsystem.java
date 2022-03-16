// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.team2930.lib.Limelight;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
  private double seesTarget;
  private double targetHeading;
  private Pose2d robotPose = new Pose2d();
  private SwerveDrivePoseEstimator estimate;
  private Pose2d limelightPose;
  private Pose2d kalmanLimelightPose;

  // private final Field2d m_field = new Field2d();
  // TODO: test and fix filtering change in the distance
  // private static double rateMetersPerSecond = 1.0;
  // private static final SlewRateLimiter distanceRateLimiter = new SlewRateLimiter(rateMetersPerSecond, 0.0);

  /** Creates a new Limelight. */
  public LimelightSubsystem(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    this.limelight = new Limelight("limelight-one");
    limelight.setPipeline(0);
    // SmartDashboard.putData("Field", m_field);
  }


  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable("limelight-one");
    pitch = table.getEntry("ty").getDouble(0);
    yaw = table.getEntry("tx").getDouble(0);
    latency = (table.getEntry("tl").getDouble(0));
    seesTarget = table.getEntry("tv").getDouble(0);

    robotPose = m_drivetrain.getPose();

    // yaw is reported negative in Counter Clockwise direction, need to reverse it.
    targetHeading = robotPose.getRotation().getDegrees() - yaw;

    SmartDashboard.putBoolean("LL has target", seesTarget==1);
    SmartDashboard.putNumber("LL pitch", pitch);

    if (seesTarget==1) {
      distance_meters = limelight.getDist(
        Units.inchesToMeters(Constants.LimelightConstants.HIGH_HUB_HEIGHT_INCHES),
        Units.inchesToMeters(Constants.LimelightConstants.LIMELIGHT_HEIGHT_INCHES),
        Constants.LimelightConstants.LIMELIGHT_PITCH_DEGREES) 
        + Units.inchesToMeters(Constants.LimelightConstants.HIGH_HUB_RADIUS_INCHES);
      SmartDashboard.putNumber("LL distance ft", Units.metersToFeet(distance_meters));
      SmartDashboard.putNumber("LL distance inches", Units.metersToInches(distance_meters));
      SmartDashboard.putNumber("LL target heading", targetHeading);
    } else {
      // return zero if we don't see the target
      distance_meters = 0;
      SmartDashboard.putNumber("LL distance ft", 0);
    }
    SmartDashboard.putNumber("LL pipelineLatency", latency);

      //TODO: should we use IMU rotation?
      //TODO: put in actual standard devs

      limelightPose = PhotonUtils.estimateFieldToRobot(
        Constants.LimelightConstants.LIMELIGHT_HEIGHT_INCHES, 
        Constants.LimelightConstants.HIGH_HUB_HEIGHT_INCHES, 
        Units.degreesToRadians(Constants.LimelightConstants.LIMELIGHT_PITCH_DEGREES), 
        Units.degreesToRadians(pitch), 
        new Rotation2d(Units.degreesToRadians(yaw)), 
        m_drivetrain.getRotation(), 
        new Pose2d(
          Constants.HubCentricConstants.HUB_CENTER_POSE2D.getX() + (Constants.LimelightConstants.HIGH_HUB_RADIUS_INCHES * Math.cos(-Units.degreesToRadians(m_drivetrain.getRotation().getDegrees()+yaw))),
          Constants.HubCentricConstants.HUB_CENTER_POSE2D.getY() + (Constants.LimelightConstants.HIGH_HUB_RADIUS_INCHES * Math.sin(-Units.degreesToRadians(m_drivetrain.getRotation().getDegrees()+yaw))), 
          Constants.HubCentricConstants.HUB_CENTER_POSE2D.getRotation()), 
        Constants.LimelightConstants.LIMELIGHT_TO_ROBOT);
      
      estimate = new SwerveDrivePoseEstimator(
        m_drivetrain.getRotation(),
        m_drivetrain.getPose(),
        m_drivetrain.kinematics(),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // State measurement standard deviations. X, Y, theta.
        new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02), // Rotation standard dev. theta.
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Vision standard devs. X, Y, and theta.
      if(seesTarget()){
        estimate.addVisionMeasurement(limelightPose, (System.currentTimeMillis() - latency)/1000);
      }
      kalmanLimelightPose = estimate.update(m_drivetrain.getRotation(), m_drivetrain.getSwerveModuleState());

      SmartDashboard.putNumber("LL pose X meters", kalmanLimelightPose.getX());
      SmartDashboard.putNumber("LL pose Y meters", kalmanLimelightPose.getY());
      SmartDashboard.putNumber("LL pose Rotation degrees", kalmanLimelightPose.getRotation().getDegrees());

    // m_field.setRobotPose(getLimelightPose());
  }

  public Pose2d getLimelightPose(){
    if(seesTarget==1){
      return kalmanLimelightPose;
    } else {
      return new Pose2d(0,0, new Rotation2d(0));
    }
  }

  public boolean seesTarget() {
    return seesTarget();
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
    return Units.degreesToRadians(targetHeading);
  }

  /** 
   * the angle offset to the vision target
   */
  public double hubRotationDegrees () {
    return yaw;
  }
}
