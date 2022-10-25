// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimbCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Automated climb program named after the former operator/person who climbed manually during the 2022 competition season, Cooper.
 * 
 * COOPER stands for: 
 * C - Climb 
 * O - Optimized 
 * O - One-Press
 * P - Program 
 * E - Executable 
 * R - Remotely 
 */
public class COOPER extends SequentialCommandGroup {
  public COOPER(ElevatorSubsystem elevator, ArmSubsystem arm, LimelightSubsystem limelight, Drivetrain drivetrain, IntakeSubsystem  intake, ShooterSubsystem shooter) {
    addCommands(
      // make sure arms are back and out of the way before climbing to Mid
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE)
        .withTimeout(0.2),

      // turn off limelight LEDs to save the drive team's eyes
      new InstantCommand(() -> limelight.turnOffAllLEDS()),

      new InstantCommand(() -> shooter.stop(), shooter),

      // Lift robot onto Mid bar. Elevator down, set to below zero hight to compensate
      // for string stretch under load. Limit switch prevents us from actually breaking 
      // the elevator.
      new MotionMagicControl(elevator, -0.5, 0.05, 0.5, 25),

      // Arms forward onto the Mid bar
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.45),

      // Extend elevator a little so we are supported by only arms.
      new MotionMagicControl(elevator, 5, 0.05, 0.5, 25),

      // Lean back. Arms full forward to lean the robot back.
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_FORWARD_ANGLE)
        .withTimeout(0.3),

      // Fully extend Elevator. //this is soft limit max
      new MotionMagicControl(elevator, 26, 0.05, 0.25, 31),

      // Let the robot settle, stop rocking so that elevator hooks are set
      new WaitCommand(0.5),

      new InstantCommand(() -> intake.deployIntake(), intake),

      new WaitCommand(0.3),

      // Set the elevator hooks by moving arms back slightly
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.4),

        new InstantCommand(() -> intake.retractIntake(), intake),

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

      //go back to break mode after coast mode 
      new InstantCommand(() -> arm.setMotorBreakMode()),

      // Arms are free of MID bar, move them back out of the way before climbing
      // to the HIGH bar
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE)
        .withTimeout(0.1),

      // Finish climb to HIGH bar
      new MotionMagicControl(elevator, -0.5, 0.05, 0.75, 10),

      // Arms on HIGH bar
      new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
        .withTimeout(0.6),

      //extend elevator up while the arms are not pushing hard against the bar 
      new ParallelCommandGroup(
        new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_HANG_ANGLE)
          .withTimeout(0.3),

        // Elevator up a little to set Arms.

        new MotionMagicControl(elevator, 13, 0.05, 0.5, 25)
      ),

      //wait for swing to settle on high
       new WaitCommand(2),

       new WaitUntilCommand(() -> (
         drivetrain.getGyroscopePitchVelocity() <= 0) && 
         (drivetrain.getGyroscopePitch() <= 0) 
       ),

       //extend all the way straight up  
       new MotionMagicControl(elevator, 25.5, 0.05, 0.25, 31),

       //lean back to hook back hooks on traversal bar 
       new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_FORWARD_ANGLE)
        .until(() -> arm.getArmAngle() >= 19),

        new InstantCommand(() -> intake.deployIntake(), intake),

      //wait for safe grab on the traversal bar 
       new WaitUntilCommand(() -> (
         drivetrain.getGyroscopePitchVelocity() >= 0) && 
         (drivetrain.getGyroscopePitch() <= -21) && 
         (arm.getArmAngle() >= 15) ),

        new WaitCommand(0.2),

        new WaitUntilCommand(() -> (
         drivetrain.getGyroscopePitchVelocity() >= 0) && 
         (drivetrain.getGyroscopePitch() <= -21) && 
         (arm.getArmAngle() >= 15) ),

      //pull down hard and fast 

      

       new MotionMagicControl(elevator, 9, 0.05, 0.25, 28),

       new InstantCommand(() -> intake.retractIntake(), intake),

       //arms back at the end of the climb 
       new ArmSetAngle(arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE)
    );
  }
}
