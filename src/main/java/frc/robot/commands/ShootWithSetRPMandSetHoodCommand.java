// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithSetRPMandSetHoodCommand extends CommandBase {
  private CargoSubsystem m_cargoSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private HoodSubsystem m_hoodSubsystem;
  private Robot m_robot;
  private long m_time;
  private double m_rpm;
  private double m_hoodAngle;
  private boolean shooting = false;

  public ShootWithSetRPMandSetHoodCommand(int flyWheelRPM, double hoodAngleDegrees, CargoSubsystem cargoSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, Robot robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cargoSubsystem = cargoSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_hoodSubsystem = hoodSubsystem;
    m_robot = robot;
    m_rpm = flyWheelRPM;
    m_hoodAngle = hoodAngleDegrees;
    m_time = 0;


    // drivetrain is not included in the requirements, as it use in a "read only"
    // fashion, to call getPose(). 
    addRequirements(cargoSubsystem, shooterSubsystem, hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //by default from testing on 2/26 2000 works well enough for low goal shots 
    
    //m_rpm = SmartDashboard.getNumber("AAA shooting rpm testing", 2000);
    SmartDashboard.putNumber("SHOOTING RPM", m_rpm);

    SmartDashboard.putString("AAAA Shoot Command State", "Initialized");
    m_shooterSubsystem.setFlywheelRPM(m_rpm);
    m_hoodSubsystem.setDesiredAngle(m_hoodAngle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wait until flywheel is fully revved
    // once it is, set indexer in shooting mode
    SmartDashboard.putString("AAAA Shoot Command State", "execute");
    if (!shooting && m_shooterSubsystem.isAtDesiredRPM() && m_hoodSubsystem.isAtAngle()) {
      shooting = true;
      SmartDashboard.putBoolean("AAA can shoot", true);
      m_cargoSubsystem.setShootMode();
    }
    SmartDashboard.putBoolean("AAA shooting", shooting);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("AAAA Shoot Command State", "end");
    shooting = false;
    m_shooterSubsystem.stop();
    m_cargoSubsystem.setStopMode();
    SmartDashboard.putBoolean("AAA shooting", false);
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
