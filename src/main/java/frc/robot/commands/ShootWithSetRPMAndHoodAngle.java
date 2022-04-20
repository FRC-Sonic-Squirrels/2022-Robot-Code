// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithSetRPMAndHoodAngle extends CommandBase {
  private CargoSubsystem m_cargoSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private HoodSubsystem m_hoodSubsystem;
  private long m_time;
  private double m_rpm;
  private double m_hoodAngle;
  private boolean shooting = false;
  private Robot m_robot;

  public ShootWithSetRPMAndHoodAngle(double flyWheelRPM, double hoodAngleSupplier, CargoSubsystem cargoSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, Robot robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cargoSubsystem = cargoSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_hoodSubsystem = hoodSubsystem;
    m_rpm = flyWheelRPM;
    m_hoodAngle = hoodAngleSupplier;
    m_time = 0;
    m_robot = robot;

    addRequirements(cargoSubsystem, shooterSubsystem, hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    m_shooterSubsystem.setFlywheelRPM(m_rpm);

    m_hoodSubsystem.setAngleDegrees(m_hoodAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wait until flywheel is fully revved
    // once it is, set indexer in shooting mode
    if (!shooting && m_shooterSubsystem.isAtDesiredRPM() && m_hoodSubsystem.isAtAngle()) {
      shooting = true;
      m_cargoSubsystem.setShootMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooting = false;
    m_time = 0;
    m_cargoSubsystem.setIdleMode();
    if (!m_robot.isAutonomous()) {
      m_shooterSubsystem.idle();
      m_hoodSubsystem.setMinAngle();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Command will stop when all the cargo are gone
    //if (m_robot.isAutonomous()) {
      if ((!m_cargoSubsystem.cargoInUpperBelts()) && (!m_cargoSubsystem.cargoInLowerBelts())) {
        if (m_time == 0) {
          m_time = System.currentTimeMillis();
        } else if (System.currentTimeMillis() - m_time >= 400) {
          return true;
        }
      }
      if (m_cargoSubsystem.cargoInUpperBelts() || m_cargoSubsystem.cargoInLowerBelts()) {
        // reset timer if we see a cargo in the indexer
        m_time = 0;
      }
    //}
    // the command will be manually executed and ended by holding a button in teleop
    return false;
  }
}
