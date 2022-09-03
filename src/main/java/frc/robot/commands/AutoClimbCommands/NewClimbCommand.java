// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimbCommands;

import com.team2930.lib.Limelight;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewClimbCommand extends SequentialCommandGroup {
  /** Creates a new NewClimbCommand. */
  public NewClimbCommand(ElevatorSubsystem elevator, ArmSubsystem arm, LimelightSubsystem limelight) {
    addCommands(
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE)
        .withTimeout(0.1),

      new InstantCommand(() -> limelight.turnOffAllLEDS()),

      new MotionMagicControl(elevator, -0.5, 0.05, 0.5, 25),

      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.3),

      new MotionMagicControl(elevator, 5, 0.05, 0.5, 25),

      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_FORWARD_ANGLE)
        .withTimeout(0.25),

      new MotionMagicControl(elevator, 24.68, 0.05, 0.5, 18),

      new WaitCommand(0.5),

      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.25),

      new WaitCommand(0.5),

      new MotionMagicControl(elevator, 19, 0.05, 0.5, 15),

      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_FORWARD_ANGLE)
        .withTimeout(0.25),

      new MotionMagicControl(elevator, 15, 0.05, 0.75, 10),


      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE)
        .withTimeout(0.1),

     new MotionMagicControl(elevator, -0.5, 0.05, 0.75, 10),

      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.3),

      new MotionMagicControl(elevator, 5, 0.05, 0.5, 25)
    );
  }
}
