// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.CargoMoveToUpperBeltsCommand;
import frc.robot.commands.ShootOneCargoCommand;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// this class is for the second basic auton command:
// shooting current cargo then moving out of the tarmac
public class SimpleAutonCommandTwo {

  private static Drivetrain m_drivetrain;
  private static TestTrajectories m_tt;
  private static CargoSubsystem m_cargo;
  private static ShooterSubsystem m_shooter;
  private static IntakeSubsystem m_intake;

  public SimpleAutonCommandTwo (Drivetrain drive, CargoSubsystem cargo, ShooterSubsystem shoot, IntakeSubsystem intake,
      double maxV, double maxA, boolean isSwerve) {

    m_drivetrain = drive;
    m_tt = new TestTrajectories(maxV, maxA, drive, isSwerve);
    m_cargo = cargo;
    m_shooter = shoot;
    m_intake = intake;

  }

   /**
   * command for autonomously shooting and then moving out of the tarmac.
   * @param startPos the starting position of the robot (assumed to be facing the hub)
   * @param targetPos the location the robot will move
   * @return a set of actions with the robot shooting its current cargo, then moving out of the tarmac
   */
  public static Command shootAndMoveOutOfTarmacCommand(Pose2d startPos, Translation2d targetPos) {

    m_drivetrain.setPose(startPos, startPos.getRotation());

    //startToShoot.transformBy(new Transform2d(new Pose2d(shootPos.getX(), shootPos.getY(), shootAngle), drivetrain.getPose()));

    return new SequentialCommandGroup(
      // 1. move cargo to upper belts, set up flywheel, then move to the shooting location
      new ParallelCommandGroup(
        new CargoMoveToUpperBeltsCommand(m_cargo),
        new WaitUntilCommand(() -> m_cargo.cargoInUpperBelts()),
        new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_activated)),
        new WaitUntilCommand(() -> m_shooter.isAtDesiredRPM())
      ),
      
      // 2. shoot the cargo
      new ShootOneCargoCommand(m_cargo, m_shooter, m_intake), 
      
      // 3. slow down flywheel and move away
      new ParallelCommandGroup(
        new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_idle)),
        SwerveControllerCommand(m_tt.driveToPose(poseToTranslation(startPos), targetPos), true)
      )
    );

  }

    /**
   * Create a swerve trajectory follow command. If stopAtEnd is set to true, robot will come to full
   * stop at the end of the command.
   * 
   * @param trajectory
   * @param drivetrain
   * @param stopAtEnd
   * @return trajectoryFollowCommand
   */
  public static Command SwerveControllerCommand(Trajectory trajectory, boolean stopAtEnd) {

    // Example from WPILib:
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/RobotContainer.java

    var thetaController =
        new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController,
            AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Command swerveControllerCommand = new SwerveControllerCommand(trajectory,
        m_drivetrain::getPose, m_drivetrain.kinematics(),
        new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
        new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD), thetaController,
        m_drivetrain::setModuleStates, m_drivetrain);

    if (stopAtEnd) {
      // Stop at the end. A good safe default, but not desireable if running two paths back to back
      swerveControllerCommand = swerveControllerCommand
          .andThen(() -> m_drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
    }

    
    return swerveControllerCommand;
  }

  private static Translation2d poseToTranslation(Pose2d pose) {
    return new Translation2d(pose.getX(), pose.getY());
  }


    
}
