// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.CargoMoveToUpperBeltsCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.ShootOneCargoCommand;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// this class is for the third basic auton command:
// shooting the current cargo, then moving out of tarmac to collect next cargo
public class SimpleAutonCommandThree {

  private static Drivetrain m_drivetrain;
  private static TestTrajectories m_tt;
  private static CargoSubsystem m_cargo;
  private static ShooterSubsystem m_shooter;
  private static IntakeSubsystem m_intake;

  public SimpleAutonCommandThree (Drivetrain drive, CargoSubsystem cargo, ShooterSubsystem shoot, IntakeSubsystem intake,
      double maxV, double maxA, boolean isSwerve) {

    m_drivetrain = drive;
    m_tt = new TestTrajectories(maxV, maxA, drive, isSwerve);
    m_cargo = cargo;
    m_shooter = shoot;
    m_intake = intake;

  }

   /**
   * command for autonomously shooting and then moving to the next set of cargo coordinates.
   * @param startPos the robot's starting position (assumed to be facing the hub)
   * @param cargoPos the position of the wanted cargo 
   * @return a set of actions with the robot shooting its current cargo, then moving and picking up the next cargo
   */
  public static Command shootAndMoveToCargoCommand(Pose2d startPos, Translation2d cargoPos) {

    m_drivetrain.setPose(startPos, startPos.getRotation());

    Translation2d midPos = midPosFinder(poseToTranslation(startPos), cargoPos);
    Pose2d rot_midPos = new Pose2d(midPos, getTranslationsAngle(poseToTranslation(startPos), cargoPos));

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
      
      // 3. slow down flywheel, deploy the intake, then move in front of the next cargo
      new ParallelCommandGroup(
        new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_idle)),
        SwerveControllerCommand(m_tt.driveToPose(startPos, rot_midPos), true),
        new IntakeDeployCommand(m_intake, m_cargo),
        new WaitUntilCommand(() -> m_intake.intakeAtDesiredRPM())
      ),
      
      // 4. collect the cargo, then wait until it is fully in the lower belts
      SwerveControllerCommand(m_tt.driveToPose(midPos, cargoPos), true),
      new WaitUntilCommand(() -> m_cargo.cargoInLowerBelts()),

      // 5. retract and deactivate the intake
      new ParallelCommandGroup(
        new InstantCommand(() -> m_intake.retractIntake()),
        new InstantCommand(() -> m_intake.stop())
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


  // private method that gets the angle between two Translation2ds
  private static Rotation2d getTranslationsAngle(Translation2d pose1, Translation2d pose2) {
    Vector2d vector1 = new Vector2d(pose1.getX(), pose1.getY());
    Vector2d vector2 = new Vector2d(pose2.getX(), pose2.getY());
    double dotProduct = vector1.dot(vector2);
    double magnitude = vector1.magnitude() * vector2.magnitude();

    return new Rotation2d( Math.acos(dotProduct/magnitude) );
  }

  // private method that gets a midPos between two poses
  private static Translation2d midPosFinder(Translation2d start, Translation2d target) {
    //double hypotenuse = Math.hypot( (target.getX() - start.getX()), (target.getY() - start.getY()) );
    double xDist = (target.getX() - start.getX()) * 0.75;
    double yDist = (target.getY() - start.getY()) * 0.75;
    Translation2d midPos = new Translation2d(start.getX() + xDist, start.getY() + yDist);

    return midPos;
  }

  private static Translation2d poseToTranslation(Pose2d pose) {
    return new Translation2d(pose.getX(), pose.getY());
  }


    
}
