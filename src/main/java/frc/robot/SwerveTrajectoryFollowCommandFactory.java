// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.CargoMoveToUpperBeltsCommand;
import frc.robot.commands.ShootOneCargoCommand;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This is a Swerve Trajectory Follow Command Factory, a helper class to create a command to follow
 * a given trajectory.
 * 
 */
public class SwerveTrajectoryFollowCommandFactory {

  /**
   * Create a swerve trajectory follow command. If stopAtEnd is set to true, robot will come to full
   * stop at the end of the command.
   * 
   * @param trajectory
   * @param drivetrain
   * @param stopAtEnd
   * @return trajectoryFollowCommand
   */
  public static Command SwerveControllerCommand(Trajectory trajectory,
      Drivetrain drivetrain, boolean stopAtEnd) {

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


  public static Command straightForward1mCommand(TestTrajectories testTrajectories, Drivetrain drivetrain) {
    return SwerveControllerCommand(testTrajectories.straightForward(1.0), drivetrain, true);
  }

  public static Command straightForward2mCommand(TestTrajectories testTrajectories, Drivetrain drivetrain) {
    return SwerveControllerCommand(testTrajectories.straightForward(2.0), drivetrain, true);
  }

  public static Command straightBack1mCommand(TestTrajectories testTrajectories, Drivetrain drivetrain) {
    return SwerveControllerCommand(testTrajectories.straightForward(-1.0), drivetrain, true);
  }

  public static Command sidewaysLeft1mCommand(TestTrajectories testTrajectories, Drivetrain drivetrain) {
    return SwerveControllerCommand(testTrajectories.straightSideways(1.0), drivetrain, true);
  }

  public static Command figureEightCommand(TestTrajectories testTrajectories, Drivetrain drivetrain) {
    return SwerveControllerCommand(testTrajectories.figureEight(0.5), drivetrain, true);
  }

  public static Command curveLeftCommand(TestTrajectories testTrajectories, Drivetrain drivetrain) {
    return SwerveControllerCommand(testTrajectories.simpleCurve(1.0, 1.0), drivetrain, true);
  }

  public static Command curveRightCommand(TestTrajectories testTrajectories, Drivetrain drivetrain) {
    return SwerveControllerCommand(testTrajectories.simpleCurve(1.0, -1.0), drivetrain, true);
  }

  public static Command doNothingCommand(Drivetrain drivetrain) {
      return new InstantCommand(() -> drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
  }

  /**
   * Adds Test Trajectories to chooser. The user still needs to add this chooser to smart dashboard:
   * 
   *    SendableChooser<Command> chooser = new SendableChooser<>();
   *    SwerveTrajectoryFollowCommandFactory.addTestTrajectoriesToChooser(chooser, 1.0, 0.75, drivetrain, true);
   *    SmartDashboard.putData("Auto mode", chooser);
   * 
   * @param chooser
   */
  public static void addTestTrajectoriesToChooser(SendableChooser<Command> chooser, double maxVelocity, double maxAcceleration, Drivetrain drivetrain, boolean isSwerve) {

    TestTrajectories tt = new TestTrajectories(maxVelocity, maxAcceleration, drivetrain, isSwerve);

    chooser.addOption("Figure 8", figureEightCommand(tt, drivetrain));
    chooser.addOption("Go Forward 1m", straightForward1mCommand(tt, drivetrain));
    chooser.addOption("Go Forward 2m", straightForward2mCommand(tt, drivetrain));
    chooser.addOption("Go Back 1m", straightBack1mCommand(tt, drivetrain));
    chooser.addOption("Curve Left", curveLeftCommand(tt, drivetrain));
    chooser.addOption("Curve Right", curveRightCommand(tt, drivetrain));
    if (isSwerve) {
      chooser.addOption("Go Sideways 1m", sidewaysLeft1mCommand(tt, drivetrain));
    }
    chooser.setDefaultOption("Do Nothing", doNothingCommand(drivetrain));
  }

  public static Command getOutOfTarmacAutonomousCommand(TestTrajectories testTrajectories, Drivetrain drivetrain) {
    //TODO: figure out actual positions (add into constants)
    Pose2d startPos = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d outOfTarmac = new Pose2d(10, 10, new Rotation2d(0));
    var trajectory = testTrajectories.simpleCurve(outOfTarmac.getX() - startPos.getX(), outOfTarmac.getY() - startPos.getY());
    // if you want to add more commands, add to this sequential command group
    return new SequentialCommandGroup( SwerveControllerCommand(trajectory, drivetrain, true) );
  }

  public static Command shootFromTarmacAutonomousCommand(TestTrajectories testTrajectories,
    Drivetrain drivetrain, CargoSubsystem cargo,
    ShooterSubsystem shooter, IntakeSubsystem intake) {
      Pose2d startPos = new Pose2d(0, 0, new Rotation2d(0));
      //Pose2d shootPos = new Pose2d(0, 0, new Rotation2d(0));
      Pose2d outOfTarmac = new Pose2d(10, 10, new Rotation2d(0));

      double rpm = 0;
      
      //var TrajectoryToShootPos = testTrajectories.simpleCurve(shootPos.getX() - startPos.getX(), shootPos.getY() - startPos.getY());
      var trajectoryToOutOfTarmac = testTrajectories.simpleCurve(outOfTarmac.getX() - startPos.getX(), outOfTarmac.getY() - startPos.getY());

      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          new CargoMoveToUpperBeltsCommand(cargo),
          new WaitUntilCommand(() -> cargo.cargoInUpperBelts()),
          new InstantCommand(() -> shooter.setFlywheelRPM(rpm)),
          new WaitUntilCommand(() -> shooter.isAtDesiredRPM())
          //SwerveControllerCommand(TrajectoryToShootPos, drivetrain, true)
        ),
        new ShootOneCargoCommand(cargo, shooter, intake),
        new WaitCommand(0.1),
        SwerveControllerCommand(trajectoryToOutOfTarmac, drivetrain, true)
      );
  }

  public static Command shootAndMoveToCargo(TestTrajectories testTrajectories,
    Drivetrain drivetrain, ShooterSubsystem shooterSubsystem, CargoSubsystem cargo, IntakeSubsystem intake) {

      // get actual coordinates of the cargo
      Pose2d startPos = drivetrain.getPose();
      Pose2d midPos = new Pose2d(10, 10, new Rotation2d(0));
      Pose2d cargoPos = new Pose2d(10, 12, new Rotation2d(0));

      Transform2d startToMid = new Transform2d(startPos, midPos);

      double rpm = 0;


      

    }
}
