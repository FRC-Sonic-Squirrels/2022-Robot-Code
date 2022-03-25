// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LimelightRotateToHubAndShoot extends CommandBase {
  private LimelightSubsystem m_limelight;
  private Drivetrain m_drivetrain;
  private CargoSubsystem m_cargoSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private HoodSubsystem m_hoodSubsystem;
  private ProfiledPIDController rotationalController = new ProfiledPIDController(3.0, 0.0, 0.02,
      new TrapezoidProfile.Constraints(
          Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5,
          Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.4));
  private double m_targetYaw;
  private double m_rotationCorrection;
  private double m_time;
  private double m_targetAngle;

  private double target_distance_meters = 0.0;
  private double target_rpm = 2000;

  private boolean shooting = false;

  private boolean setDistance = true;
  /** Creates a new VisionTurnToHub. */
  public LimelightRotateToHubAndShoot(LimelightSubsystem limelight, Drivetrain drivetrain, CargoSubsystem cargoSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    m_drivetrain = drivetrain;
    m_cargoSubsystem = cargoSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_hoodSubsystem = hoodSubsystem;
    m_time = 0;

    addRequirements(cargoSubsystem, shooterSubsystem, hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public boolean isAtTargetAngle(){
    return Math.abs(m_drivetrain.getRotation().getDegrees() - m_targetAngle) < 2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.seesTarget()) {
      m_targetYaw = Math.toRadians(m_limelight.hubRotationDegrees());
      m_targetAngle = m_drivetrain.getPose().getRotation().getRadians() + m_targetYaw;
      m_rotationCorrection =
          rotationalController.calculate(m_drivetrain.getRotation().getRadians(), m_targetAngle);
    
      m_rotationCorrection *= 0.3;

      if(isAtTargetAngle()){
        if(setDistance){
          target_distance_meters = m_limelight.getDistanceMeters();
          setDistance = false;
        }
        if(Math.abs(m_limelight.getDistanceMeters()-target_distance_meters) > 0.5){
          setDistance = true;
        }
      
        target_rpm = m_shooterSubsystem.getRPMforDistanceFeet(Units.metersToFeet(target_distance_meters));
        m_targetAngle = m_hoodSubsystem.getAngleForDistance(Units.metersToFeet(target_distance_meters));
        m_shooterSubsystem.setFlywheelRPM(target_rpm);
        m_hoodSubsystem.setDesiredAngle(m_targetAngle);
        if (!shooting && m_shooterSubsystem.isAtDesiredRPM() & m_hoodSubsystem.isAtAngle()) {
          shooting = true;
          m_cargoSubsystem.setShootMode();
        }
      }
    }
    SmartDashboard.putBoolean("SHOOTING", shooting);
    SmartDashboard.putBoolean("LL facing toward hub", isAtTargetAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_cargoSubsystem.setStopMode();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command will stop when all the cargo are gone
    if ((! m_cargoSubsystem.cargoInUpperBelts()) && (! m_cargoSubsystem.cargoInLowerBelts())) {
      if (m_time == 0) {
        m_time = System.currentTimeMillis();
      }
      else if (System.currentTimeMillis() - m_time >= 1000) {
        return true;
      }
    }
    if (m_cargoSubsystem.cargoInUpperBelts() || m_cargoSubsystem.cargoInLowerBelts()) {
      // reset timer if we see a cargo in the indexer
      m_time = 0;
    }

    //the command will be manually executed and ended by holding a button in teleop
    return false;
  }

  

}
