// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAutoCommand extends CommandBase {
  /** Creates a new ShootOneCargoCommand. */
  private CargoSubsystem m_cargoSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private Drivetrain m_drivetrain;

  public ShootAutoCommand(CargoSubsystem cargoSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cargoSubsystem = cargoSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_drivetrain = drivetrain;

    //TODO: is drivetrain required in "add requirements"?
    addRequirements(cargoSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  // todo: What if we have a ball in the lower belts but not upper? Move it up first then shoot?
  // This command will only be called in autonomous mode, where the ball is already moved to the upper belts.
  @Override
  public void initialize() {

    //TODO: change to limelight calculated distance, rather than odometry
    m_shooterSubsystem.setFlywheelRPM(m_shooterSubsystem.getRPMforDistanceFeet(
      Math.sqrt(
        Math.pow(Constants.HubCentricConstants.HUB_CENTER_POSE2D.getX() - m_drivetrain.getPose().getX(), 2) + 
        Math.pow(Constants.HubCentricConstants.HUB_CENTER_POSE2D.getY() - m_drivetrain.getPose().getY(), 2))));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wait until flywheel is fully revved
    // once it is, turn on upper cargo belt
    // once upper ball has been released, go back to intake mode

    if (m_shooterSubsystem.isAtDesiredRPM()) {
        m_cargoSubsystem.setUpperOnlyMode();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setFlywheelRPM(m_shooterSubsystem.getIdleRPM());

    if (m_intakeSubsystem.isDeployed()) {
      m_cargoSubsystem.setIntakeMode();
    }
    else {
      m_cargoSubsystem.setStopMode();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (! m_cargoSubsystem.cargoInUpperBelts()) {
        new WaitCommand(0.5);
        return true;
      }
      return false;
  }
}