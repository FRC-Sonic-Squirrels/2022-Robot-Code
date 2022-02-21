// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.HubCentricConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveHubCentricCommand extends CommandBase {
  private Drivetrain m_drivetrain;
  private SwerveDriveKinematics m_kinematics;

  private Supplier<Double> m_sidewaysSupplier;
  private Supplier<Double> m_forwardSupplier;

  

  // copied values from Swerve Template Odometry
  private ProfiledPIDController rotationalController = new ProfiledPIDController(3.0, 0.0, 0.02,
      new TrapezoidProfile.Constraints(
          Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));


  private Vector2d m_hubCenter = Constants.HubCentricConstants.HUB_CENTER;
  
  public DriveHubCentricCommand(Drivetrain drivetrain, Supplier<Double> sidewaysSupplier, Supplier<Double> forwardSupplier) {
    m_sidewaysSupplier = sidewaysSupplier;
    m_forwardSupplier = forwardSupplier;
    m_drivetrain = drivetrain;
    m_kinematics = m_drivetrain.kinematics();

    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    //TODO: do we need continous input? drive with set rotation doesnt have it
    rotationalController.enableContinuousInput(-Math.PI, Math.PI);
    rotationalController.setTolerance(Math.PI/180); //1 degree of wiggle room

    //TODO: what does this do? drive with set rotation has this 
    rotationalController.reset(m_drivetrain.getGyroscopeRotation().getRadians());
  }

  @Override
  public void execute() {
    Rotation2d currentHeading = m_drivetrain.getGyroscopeRotation(); 
    Pose2d robotPosition = m_drivetrain.getPose();
    Vector2d robotVector = new Vector2d(m_hubCenter.x - robotPosition.getX(), m_hubCenter.y - robotPosition.getY());

    //add pi to make the back face the hub (shooter is on the back of the robot)
    Rotation2d targetHeading = getTargetHeading(robotVector, new Vector2d(1,0)).plus(new Rotation2d(Math.PI));
    //to make it work on left side of circle 
    targetHeading.times(Math.signum(m_hubCenter.y - robotPosition.getY()));
    double radius = Math.sqrt(Math.pow(m_hubCenter.x - robotPosition.getX(), 2) + Math.pow(m_hubCenter.y - robotPosition.getY(), 2));

    //Multiply by max velocity to hopefully speed up the rotation of the robot 
    double rotationCorrection = rotationalController.calculate(currentHeading.getRadians(), targetHeading.getRadians()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    double strafeX = 0.0;
    double strafeY = 0.0;

    if(m_forwardSupplier.get() != 0.0) {
      // forward/reverse is just orthogonal to tangent
      double orthogonalHeading = targetHeading.getRadians() - (Math.PI / 2.0);
      //Not scaling by radius anymore
      strafeX += findStrafeX(orthogonalHeading, Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_forwardSupplier.get(), HubCentricConstants.FORWARD_MULTIPLIER);
      strafeY += findStrafeY(orthogonalHeading, Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_forwardSupplier.get(), HubCentricConstants.FORWARD_MULTIPLIER);
    }
    if(m_sidewaysSupplier.get() != 0.0) {
       //Not scaling by radius anymore
      strafeX += findStrafeX(targetHeading.getRadians(), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_sidewaysSupplier.get(), HubCentricConstants.SIDEWAYS_MULTIPLIER);
      strafeY += findStrafeY(targetHeading.getRadians(), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_sidewaysSupplier.get(), HubCentricConstants.SIDEWAYS_MULTIPLIER);
    }
    
    //TODO: might have to adjust rotationCorrection check since we are now multiplying by max angular speed 
    if (rotationCorrection < 0.03 && strafeX < 0.01 && strafeY < 0.01) {
        // don't try to correct small turns if we aren't moving
        rotationCorrection = 0.0;
    }

    //TODO: try testing without checking for movement 
    // if (rotationCorrection < 0.03) {
    //   rotationCorrection = 0.0;
    // }

    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(strafeX, strafeY, rotationCorrection, currentHeading));

    //TODO: check if need to flip order of coordinates from x,y to y,x
    SmartDashboard.putNumber("Hub_Centric currentHeading", currentHeading.getDegrees());
    SmartDashboard.putNumber("Hub_Centric targetHeading", targetHeading.getDegrees());
    SmartDashboard.putNumberArray("Hub_Centric robotPosition", new double[] {robotPosition.getX(), robotPosition.getY()});
    
    SmartDashboard.putNumber("Hub_Centric rotationCorrection", rotationCorrection);
    SmartDashboard.putNumberArray("Hub_Centric strafe values", new double[] {strafeX, strafeY});

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

  private double findStrafeX(double targetAngle, double max_velocity, double joystickInput, double constant) {
    return constant * max_velocity * joystickInput * -Math.sin(targetAngle);
  }

  private double findStrafeY(double targetAngle, double max_velocity, double joystickInput, double constant) {
    return constant * max_velocity * joystickInput * Math.cos(targetAngle);
  }
}
