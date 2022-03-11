	// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ejml.equation.MatrixConstructor;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class VisionGetActualRobotPose extends CommandBase {
  /** Creates a new GetActualRoboPose. */
  private Drivetrain m_drivetrain;
  private NetworkTable table;
  private double pitch;
  private double target;
  private double rotation;
  private Pose2d roboPose;
  private final SwerveDriveOdometry m_odometry;
  private Matrix stdDevs = Matrix<>(Nat.N3(), Nat.N1());
  private Matrix encoderStdDevs = Matrix<>(Nat.N3(), Nat.N1());
  private Matrix visionStdDevs;
  private SwerveDrivePoseEstimator estimator;
  public VisionGetActualRobotPose(Drivetrain drivetrain, SwerveDriveOdometry odometry) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_odometry = odometry;
    table = NetworkTableInstance.getDefault().getTable("limelight");
    pitch = table.getEntry("ty").getDouble(0);
    target = table.getEntry("tv").getDouble(0);
    rotation = table.getEntry("ts").getDouble(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    roboPose = PhotonUtils.estimateFieldToRobot(
    Units.inchesToMeters(Constants.VisionConstants.CAMERA_HEIGHT_INCHES), 
    Units.inchesToMeters(Constants.VisionConstants.TARGET_HEIGHT_INCHES), 
    Units.degreesToRadians(Constants.VisionConstants.CAMERA_PITCH_DEGREES), 
    pitch, 
    Rotation2d.fromDegrees(rotation), 
    m_drivetrain.getGyroscopeRotation(),
    new Pose2d(
      Constants.VisionConstants.HIGH_HUB_RADIUS_FEET * (Math.cos(-(m_drivetrain.getGyroscopeRotation().getRadians()+Units.degreesToRadians(rotation)))), 
      Constants.VisionConstants.HIGH_HUB_RADIUS_FEET * (Math.sin(-(m_drivetrain.getGyroscopeRotation().getRadians()+Units.degreesToRadians(rotation)))),
      new Rotation2d(0)),
    Constants.VisionConstants.CAMERA_TO_ROBOT
    );
    estimator = new SwerveDrivePoseEstimator(m_drivetrain.getGyroscopeRotation(),
    m_odometry.getPoseMeters(),
    m_drivetrain.kinematics(),
    new Matrix<>(Nat.N3(), Nat.N1()).fill(0.01), // State measurement standard deviations. X, Y, theta.
    new Matrix<>(Nat.N3(), Nat.N1()).fill(0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
    new Matrix<>(Nat.N3(), Nat.N1()).fill(0.01)); // Global measurement standard deviations. X, Y, and theta.
    
    estimator.update(m_drivetrain.getGyroscopeRotation(), );
    if(target==1){
      //TODO: use Kalman filter instead: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html
      //m_drivetrain.resetOdometry(roboPose);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
