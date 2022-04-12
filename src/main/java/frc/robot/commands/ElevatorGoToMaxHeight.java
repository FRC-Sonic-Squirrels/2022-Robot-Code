// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.Sides;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToMaxHeight extends CommandBase {
  private ElevatorSubsystem m_elevator;
  private Drivetrain m_drivetrain;

  // rate of robot pitch (forward/backward) in degrees per second
  private double lastRobotPitchRate = 0;

  public ElevatorGoToMaxHeight(ElevatorSubsystem elevator, Drivetrain drivetrain) {

    m_elevator = elevator;
    m_drivetrain = drivetrain;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastRobotPitchRate = m_drivetrain.getGyroscopePitchVelocity();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentRobotPitchRate = m_drivetrain.getGyroscopePitchVelocity();

    if ((Math.signum(currentRobotPitchRate) <= 0.0) &&
       (Math.signum(lastRobotPitchRate) >= 0.0) &&
       m_drivetrain.getGyroscopePitch() > 2.0) {
      m_elevator.brakeOff();
      m_elevator.setWinchPercentOutput(-1);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_elevator.brakeOn();
      m_elevator.setWinchPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.m_atMaxheight;
  }
}
