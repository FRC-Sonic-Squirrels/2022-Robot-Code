// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// this class is for the first basic auton command:
// simply moving out of the tarmac
public class SimpleAutonCommandOne {

  private TestTrajectories m_tt;
  private Drivetrain drivetrain = null;
  private CargoSubsystem cargo = null;
  private ShooterSubsystem shoot = null;
  private IntakeSubsystem intake = null;

  public SimpleAutonCommandOne (Drivetrain drive, CargoSubsystem cargo, ShooterSubsystem shoot, IntakeSubsystem intake,
      double maxV, double maxA, boolean isSwerve) {

        this.drivetrain = drive;
        this.cargo = cargo;
        this.intake = intake;
        this.shoot = shoot;

    m_tt = new TestTrajectories(maxV, maxA, drive, isSwerve);

  }

   /**
   * command for autonomously moving out of the tarmac.
   * @param startPos the starting position of the robot (assumed to be facing the hub)
   * @param targetPos the location the robot will move
   * @return a set of actions with the robot leaving the tarmac
   */
  public Command moveOutOfTarmacCommand(Pose2d startPos, Translation2d targetPos) {

    //drivetrain.setPose(startPos, startPos.getRotation());
    Trajectory startToTarget = m_tt.driveToPose(poseToTranslation(startPos), targetPos);

    return SwerveControllerCommand(startToTarget, true);

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
  public Command SwerveControllerCommand(Trajectory trajectory, boolean stopAtEnd) {

    // Example from WPILib:
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/RobotContainer.java

    var thetaController =
        new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController,
            AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Command swerveControllerCommand = new SwerveControllerCommand(trajectory,
        drivetrain::getPose, drivetrain.kinematics(),
        new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
        new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD), thetaController,
        drivetrain::setModuleStates, drivetrain);

    if (stopAtEnd) {
      // Stop at the end. A good safe default, but not desireable if running two paths back to back
      swerveControllerCommand = swerveControllerCommand
          .andThen(() -> drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
    }

    
    return swerveControllerCommand;
  }

  private Translation2d poseToTranslation(Pose2d pose) {
    return new Translation2d(pose.getX(), pose.getY());
  }
    
}
