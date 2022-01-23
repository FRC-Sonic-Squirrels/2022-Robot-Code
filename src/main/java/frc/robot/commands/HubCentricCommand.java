// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class HubCentricCommand extends CommandBase {
  /** Creates a new HubCentricCommand. */
  Drivetrain m_drivetrain;
  Supplier<Double> m_sidewaysSupplier;
  Supplier<Double> m_forwardSupplier;
  // copied from Swerve Template Odometry
  PIDController rotationalController = new PIDController(3.0, 0.0, 0.02);

  Vector2d m_hubCenter = Constants.HUB_CENTER;
  
  public HubCentricCommand(Drivetrain drivetrain, Supplier<Double> sidewaysSupplier, Supplier<Double> forwardSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_sidewaysSupplier = sidewaysSupplier;
    m_forwardSupplier = forwardSupplier;
    m_drivetrain = drivetrain;

    addRequirements(drivetrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    rotationalController.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Rotation2d currentHeading = m_drivetrain.getGyroscopeRotation();
    Pose2d robotPosition = m_drivetrain.getCurrentPosition();
    Vector2d robotVector = new Vector2d(robotPosition.getX(), robotPosition.getY());
    Rotation2d targetHeading = orientRobot(robotVector, m_hubCenter);
    double radius = Math.sqrt( Math.pow(m_hubCenter.x - robotPosition.getX(), 2) + Math.pow(m_hubCenter.y - robotPosition.getY(), 2) );

    double rotationCorrection = rotationalController.calculate(currentHeading.getRadians(), targetHeading.getRadians());

    // TODO: Check if controller needs to be multiplied by max velocity
    double strafeX = findStrafeX(radius, targetHeading.getRadians(), m_drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_sidewaysSupplier.get(), 0.04);
    double strafeY = findStrafeY(radius, targetHeading.getRadians(), m_drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_forwardSupplier.get(), 0.04);

    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(strafeY, strafeX, rotationCorrection, currentHeading));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Rotation2d orientRobot(Vector2d robotLocation, Vector2d hubLocation) {

    double product = robotLocation.dot(hubLocation);
    double magnitudes = robotLocation.magnitude() * hubLocation.magnitude();
    double angle_rad = Math.acos(product / magnitudes);

    return Rotation2d.fromDegrees(Math.toDegrees(angle_rad));

  }

  private double findStrafeX(double radius, double targetAngle, double max_velocity, double joystickInput, double constant) {

    return max_velocity * radius * joystickInput * constant * -Math.sin(targetAngle);

  }

  private double findStrafeY(double radius, double targetAngle, double max_velocity, double joystickInput, double constant) {

    return max_velocity * radius * joystickInput * constant * Math.cos(targetAngle);

  }
}
