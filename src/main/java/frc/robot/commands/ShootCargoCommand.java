// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCargoCommand extends CommandBase {
  /** Creates a new ShootTwoCargoCommand. */
  private CargoSubsystem m_cargoSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private Robot m_robot;
  private long m_time;
  private double rpm = 2000;

  public ShootCargoCommand(double rpm, CargoSubsystem cargoSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, Robot robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rpm = rpm;
    m_cargoSubsystem = cargoSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_robot = robot;
    m_time = 0;

    addRequirements(cargoSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setFlywheelRPM(rpm);
    m_intakeSubsystem.deployIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wait until flywheel is fully revved
    // once it is, turn on upper cargo belt
    // once upper ball has been released, go back to intake mode

    if (m_shooterSubsystem.isAtDesiredRPM()) {
      m_cargoSubsystem.setBothMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_cargoSubsystem.setStopMode();
    m_intakeSubsystem.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // the command will automatically stop when both cargo are released in autonomous
    if (m_robot.isAutonomous()) {
      // the command will stop .5 seconds after no cargo is detected, to let the cargo finish shooting
      if ((! m_cargoSubsystem.cargoInUpperBelts()) && (! m_cargoSubsystem.cargoInLowerBelts())) {
        if (m_time == 0) {
          m_time = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() - m_time >= 500) {
          return true;
        }
      }
    }
    // the command will be manually executed and ended by holding a button in teleop
    return false;
  }
}
