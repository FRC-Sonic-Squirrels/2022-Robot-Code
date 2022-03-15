// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;

import frc.robot.subsystems.Drivetrain;


public class DriveFixedRotationToHub extends CommandBase {
  private Drivetrain m_drivetrain;
  private SwerveDriveKinematics m_kinematics;

  private Supplier<Double> m_sidewaysSupplier;
  private Supplier<Double> m_forwardSupplier;

  private ProfiledPIDController rotationalController = new ProfiledPIDController(3.0, 0.0, 0.02,
  new TrapezoidProfile.Constraints(
      Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));


private Vector2d m_hubCenter = Constants.HubCentricConstants.HUB_CENTER;
  /** Creates a new DriveFixTheHub. */
  public DriveFixedRotationToHub(Drivetrain drivetrain, Supplier<Double> sidewaysSupplier, Supplier<Double> forwardSupplier) {
    m_sidewaysSupplier = sidewaysSupplier;
    m_forwardSupplier = forwardSupplier;
    m_drivetrain = drivetrain;
    m_kinematics = m_drivetrain.kinematics();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //here input the position
  rotationalController.enableContinuousInput(-Math.PI, Math.PI);
  rotationalController.setTolerance(Math.PI/180); //1 degree of wiggle room

  
  rotationalController.reset(m_drivetrain.getRotation().getRadians());
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d currentHeading = m_drivetrain.getRotation(); 
    Pose2d robotPosition = m_drivetrain.getPose();
    Vector2d robotVector = new Vector2d(m_hubCenter.x - robotPosition.getX(), m_hubCenter.y - robotPosition.getY());
    //add pi to make the back face the hub (shooter is on the back of the robot)
    Rotation2d targetHeading = getTargetHeading(robotVector, new Vector2d(1,0)).plus(new Rotation2d(Math.PI));
    //to make it work on left side of circle 
    targetHeading.times(Math.signum(m_hubCenter.y - robotPosition.getY()));
    double radius = Math.sqrt(Math.pow(m_hubCenter.x - robotPosition.getX(), 2) + Math.pow(m_hubCenter.y - robotPosition.getY(), 2));




        //Multiply by max velocity to hopefully speed up the rotation of the robot 
        double rotationCorrection = rotationalController.calculate(currentHeading.getRadians(), targetHeading.getRadians()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        

        SmartDashboard.putNumber("Hub_Centric currentHeading", currentHeading.getDegrees());
        SmartDashboard.putNumber("Hub_Centric targetHeading", targetHeading.getDegrees());
        SmartDashboard.putNumberArray("Hub_Centric robotPosition", new double[] {robotPosition.getX(), robotPosition.getY()});
        SmartDashboard.putNumberArray("Hub_Centric hubPosition", new double[] {Constants.HubCentricConstants.HUB_CENTER_POSE2D.getX(), Constants.HubCentricConstants.HUB_CENTER_POSE2D.getY()});
    
        SmartDashboard.putNumber("Hub_Centric rotationCorrection", rotationCorrection);
    
        SmartDashboard.putNumber("Hub_Centric sidewaysInput", m_sidewaysSupplier.get());
        SmartDashboard.putNumber("Hub_Centric forwardInput", m_forwardSupplier.get());
    
        SmartDashboard.putNumber("Hub_Centric radius", radius);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  private Rotation2d getTargetHeading(Vector2d robotLocation, Vector2d hubLocation) {
    double product = robotLocation.dot(hubLocation);
    double magnitudes = robotLocation.magnitude() * hubLocation.magnitude();
    double angle_rad = Math.acos(product / magnitudes);
    return new Rotation2d(angle_rad);
}
}
