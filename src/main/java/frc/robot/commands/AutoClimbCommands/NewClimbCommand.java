// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimbCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewClimbCommand extends SequentialCommandGroup {
  /** Creates a new NewClimbCommand. */
  public NewClimbCommand(ElevatorSubsystem elevator, ArmSubsystem arm, LimelightSubsystem limelight, Drivetrain drivetrain) {
    addCommands(

      // make sure arms are back and out of the way before climbing to Mid
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE)
        .withTimeout(0.1),

      // turn off limelight LEDs to save the drive team's eyes
      new InstantCommand(() -> limelight.turnOffAllLEDS()),

      // Lift robot onto Mid bar. Elevator down, set to below zero hight to compensate
      // for string stretch under load. Limit switch prevents us from actually breaking 
      // the elevator.
      new MotionMagicControl(elevator, -0.5, 0.05, 0.5, 25),

      // Arms forward onto the Mid bar
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.3),

      // Extend elevator a little so we are supported by only arms.
      new MotionMagicControl(elevator, 5, 0.05, 0.5, 25),

      // Lean back. Arms full forward to lean the robot back.
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_FORWARD_ANGLE)
        .withTimeout(0.25),

      // Fully extend Elevator. //this is soft limit max
      new MotionMagicControl(elevator, 25.5, 0.05, 0.25, 31),

      // Let the robot settle, stop rocking so that elevator hooks are set
      new WaitCommand(0.5),

      // Set the elevator hooks by moving arms back slightly
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.25),

      // let the robot settle
      new WaitCommand(0.5),

      // Start lift robot onto HIGH bar. Elevator up a little to get
      // tension on arms and elevator hooks.
      new MotionMagicControl(elevator, 19, 0.05, 0.5, 15),

      // Arms forward to ease the transition of arms popping off in next step.
      // new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_FORWARD_ANGLE)
      //   .withTimeout(0.25),
      new InstantCommand(() -> arm.setArmPercentOutput(0.0), arm),
      new InstantCommand(() -> arm.coastMode(), arm),
      

      // Continue lifting onto HIGH bar, up until arms release from MID
      new MotionMagicControl(elevator, 15, 0.05, 0.75, 10),

      // Arms are free of MID bar, move them back out of the way before climbing
      // to the HIGH bar
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE)
        .withTimeout(0.1),

      // Finish climb to HIGH bar
      new MotionMagicControl(elevator, -0.5, 0.05, 0.75, 10),

      // Arms on HIGH bar
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.3),

      new ParallelCommandGroup(
        new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_HANG_ANGLE)
          .withTimeout(0.3),

        // Elevator up a little to set Arms.

        new MotionMagicControl(elevator, 13, 0.05, 0.5, 25)
      ),

      // //wait for swing to settle on high
      // new WaitCommand(1.5),

      new WaitUntilCommand(() -> (Math.abs(drivetrain.getGyroscopePitch() -7 ) < 5) && (drivetrain.getGyroscopePitchVelocity() <= 0)),

      // Lean back. Arms full forward to lean the robot back.
      new ParallelCommandGroup(
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_FORWARD_ANGLE)
        .withTimeout(0.25),
        // Fully extend Elevator. //this is soft limit max
      new MotionMagicControl(elevator, 25.5, 0.05, 0.25, 31)

      ),
      

      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.25),

      new WaitUntilCommand(() -> (Math.abs(drivetrain.getGyroscopePitch() + 25) < 1) && (Math.abs(drivetrain.getAccelX()) < 5) ),

      new MotionMagicControl(elevator, 15, 0.05, 0.5, 15),

       new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE)
        .withTimeout(0.25)

      // TODO:
      //   - lean back
      //   - elevator up
      //   - set elevator hooks on TRAVERSE bar
      //   - set arms on TRAVERSE bar
    );
  }
}
