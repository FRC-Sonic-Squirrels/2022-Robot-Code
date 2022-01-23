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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class HubCentricCommand extends CommandBase {
  Drivetrain m_drivetrain;
  Supplier<Double> m_sidewaysSupplier;
  Supplier<Double> m_forwardSupplier;

  // copied values from Swerve Template Odometry
  PIDController rotationalController = new PIDController(3.0, 0.0, 0.02);

  Vector2d m_hubCenter = Constants.HUB_CENTER;
  
  public HubCentricCommand(Drivetrain drivetrain, Supplier<Double> sidewaysSupplier, Supplier<Double> forwardSupplier) {
    m_sidewaysSupplier = sidewaysSupplier;
    m_forwardSupplier = forwardSupplier;
    m_drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    rotationalController.enableContinuousInput(-Math.PI, Math.PI);
    rotationalController.setTolerance(Math.PI/180); //1 degree of wiggle room
  }

  @Override
  public void execute() {
    Rotation2d currentHeading = m_drivetrain.getGyroscopeRotation();
    Pose2d robotPosition = m_drivetrain.getCurrentPosition();
    Vector2d robotVector = new Vector2d(robotPosition.getX(), robotPosition.getY());

    Rotation2d targetHeading = getTargetHeading(robotVector, m_hubCenter);
    double radius = Math.sqrt(Math.pow(m_hubCenter.x - robotPosition.getX(), 2) + Math.pow(m_hubCenter.y - robotPosition.getY(), 2));

    double rotationCorrection = rotationalController.calculate(currentHeading.getRadians(), targetHeading.getRadians());

    // TODO: Check if controller needs to be multiplied by max velocity
    double strafeX = findStrafeX(radius, targetHeading.getRadians(), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_sidewaysSupplier.get(), 0.04);
    double strafeY = findStrafeY(radius, targetHeading.getRadians(), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_forwardSupplier.get(), 0.04);

    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(strafeY, strafeX, rotationCorrection, currentHeading));

    SmartDashboard.putNumber("currentHeading", currentHeading.getDegrees());
    SmartDashboard.putNumber("targetHeading", targetHeading.getDegrees());
    SmartDashboard.putNumberArray("robotPosition", new double[] {robotPosition.getX(), robotPosition.getY()});
    
    SmartDashboard.putNumber("rotationCorrection", rotationCorrection);
    SmartDashboard.putNumberArray("strafe values", new double[] {strafeX, strafeY});

    SmartDashboard.putNumber("sidewaysInput", m_sidewaysSupplier.get());
    SmartDashboard.putNumber("forwardInput", m_forwardSupplier.get());

    SmartDashboard.putNumber("radius", radius);
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

  private double findStrafeX(double radius, double targetAngle, double max_velocity, double joystickInput, double constant) {
    return max_velocity * radius * joystickInput * constant * -Math.sin(targetAngle);
  }

  private double findStrafeY(double radius, double targetAngle, double max_velocity, double joystickInput, double constant) {
    return max_velocity * radius * joystickInput * constant * Math.cos(targetAngle);
  }
}
