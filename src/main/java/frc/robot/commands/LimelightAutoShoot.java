// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LimelightAutoShoot extends CommandBase {
  private LimelightSubsystem limelight;
  private CargoSubsystem cargoSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private HoodSubsystem hoodSubsystem;
  private Robot m_robot;
  private double time;
  private double hoodAngleDegrees;
  private double target_distance_meters = 0.0;
  private double target_rpm = 2000;
  private boolean m_gotValues = false;
  private boolean shooting = false;

  /** Creates a new VisionTurnToHub. */
  public LimelightAutoShoot(LimelightSubsystem limelight, CargoSubsystem cargoSubsystem,
      ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, Robot robot) {
    this.limelight = limelight;
    this.cargoSubsystem = cargoSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.m_robot = robot;
    time = 0;

    addRequirements(cargoSubsystem, shooterSubsystem, hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.seesTarget()) {

      target_distance_meters = limelight.getDistanceMeters();

      if (!m_gotValues) {
        target_rpm =
            shooterSubsystem.getRPMforDistanceFeet(Units.metersToFeet(target_distance_meters));
        hoodAngleDegrees =
            hoodSubsystem.getAngleForDistanceFeet(Units.metersToFeet(target_distance_meters));
        shooterSubsystem.setFlywheelRPM(target_rpm);
        hoodSubsystem.setAngleDegrees(hoodAngleDegrees);

        m_gotValues = true;
      }

      if (!shooting && m_gotValues && shooterSubsystem.isAtDesiredRPM() && hoodSubsystem.isAtAngle()) {
        shooting = true;
        cargoSubsystem.setShootMode();
      }
    }

    SmartDashboard.putNumber("LLRS target_rpm", target_rpm);
    SmartDashboard.putNumber("LLRS target hood Angle", hoodAngleDegrees);
    SmartDashboard.putNumber("LLRS target_distance", target_distance_meters);
    SmartDashboard.putBoolean("LLRS SHOOTING", shooting);
    SmartDashboard.putBoolean("LLRS facing toward hub", limelight.onTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    cargoSubsystem.setIdleMode();
    hoodSubsystem.setMinAngle();
    shooting = false;
    m_gotValues = false;
    time = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command will stop when all the cargo are gone
    if ((!cargoSubsystem.cargoInUpperBelts()) && (!cargoSubsystem.cargoInLowerBelts())) {
      if (time == 0) {
        time = System.currentTimeMillis();
      } else if (System.currentTimeMillis() - time >= 500) {
        return true;
      }
    }
    else {
      // reset timer if we see a cargo in the indexer
      time = 0;
    }

    // the command will be manually executed and ended by holding a button in teleop
    return false;
  }
}
