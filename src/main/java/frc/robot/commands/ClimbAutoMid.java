// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAutoMid extends SequentialCommandGroup {
  /** Creates a new ClimbAutoMid. */
  ElevatorSubsystem m_elevator;
  ArmSubsystem m_arm;
  XboxController m_climbController;
  Drivetrain m_drivetrain;

  public ClimbAutoMid(ElevatorSubsystem elevator, ArmSubsystem arm, XboxController climbController, Drivetrain drivetrain) {
    m_elevator = elevator;
    m_arm = arm;
    m_climbController = climbController;
    m_drivetrain = drivetrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // arms out of the way for climbing
      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_BACK_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle()),

      // lift robot up
      new InstantCommand(() -> m_elevator.setElevatorHeight(0), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight(0)),

      // arms back to center
      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle()),

      // lower elevator so hooks grab
      new InstantCommand(() -> m_elevator.setElevatorHeight(6), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight(6)),

      // shake the xbox controller?
      new ControllerRumbleCommand(m_climbController, 0.2),

      // rotate robot to get ready to grab higher bar
      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_FORWARD_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle()),

      // extend elevator to get ready to grab higher bar
      new InstantCommand(() -> m_elevator.setElevatorHeight(Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT), m_elevator),
      // why are there 2 different functions for .isAtHeight()
      new WaitUntilCommand(() -> m_elevator.isAtHeight()),

      // rotate robot so elevator can engage
      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_NEXT_BAR_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle()),

      // retract elevator
      new InstantCommand(() -> m_elevator.setElevatorHeight(0), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight())

      // note: we can retract elevator slower so swinging isnt as bad
    );
  }
  public boolean safeToClimb() {
    if (m_drivetrain.getGyroscopePitchVelocity() > Constants.AutoClimbConstants.MAX_PITCH_ACCEL) { return false; }
    if (m_drivetrain.getGyroscopePitch() > Constants.AutoClimbConstants.MAX_PITCH_DEGREE) { return false; }

    
    if (m_drivetrain.getGyroscopeRotationVelocity().getRadians() > Constants.AutoClimbConstants.MAX_YAW_ACCEL) { return false; }
    // TODO: add function to get gyro yaw
    // if (m_drivetrain.getGyroscopeYaw > Constants.AutoClimbConstants.MAX_YAW_DEGREE) { return false; }

    return true;
  }
}
