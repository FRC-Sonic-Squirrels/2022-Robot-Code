// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * This is a Swerve Trajectory Follow Command Factory, a helper class to create a command to follow
 * a given trajectory.
 * 
 */
public class SwerveTrajectoryAutonomousCommandFactory {

  /**
   * Create a swerve trajectory follow command. If stopAtEnd is set to true, robot will come to full
   * stop at the end of the command.
   * 
   * @param trajectory
   * @param Drivetrain
   * @param stopAtEnd
   * @return trajectoryFollowCommand
   */
  public static Command SwerveControllerCommand(Trajectory trajectory,
      Drivetrain Drivetrain, boolean stopAtEnd) {

    // Example from WPILib:
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/RobotContainer.java

    var thetaController =
        new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController,
            AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Command swerveControllerCommand = new SwerveControllerCommand(trajectory,
        Drivetrain::getPose, Drivetrain.kinematics(),
        new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
        new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD), thetaController,
        Drivetrain::setModuleStates, Drivetrain);

    if (stopAtEnd) {
      // Stop at the end. A good safe default, but not desireable if running two paths back to back
      swerveControllerCommand = swerveControllerCommand
          .andThen(() -> Drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
    }

    return swerveControllerCommand;
  }


  public static Command straightForward1mCommand(TestTrajectories testTrajectories, Drivetrain Drivetrain) {
    return SwerveControllerCommand(testTrajectories.straightForward(1.0), Drivetrain, true);
  }

  public static Command straightForward2mCommand(TestTrajectories testTrajectories, Drivetrain Drivetrain) {
    return SwerveControllerCommand(testTrajectories.straightForward(2.0), Drivetrain, true);
  }

  public static Command straightBack1mCommand(TestTrajectories testTrajectories, Drivetrain Drivetrain) {
    return SwerveControllerCommand(testTrajectories.straightForward(-1.0), Drivetrain, true);
  }

  public static Command sidewaysLeft1mCommand(TestTrajectories testTrajectories, Drivetrain Drivetrain) {
    return SwerveControllerCommand(testTrajectories.strafeSideways(1.0), Drivetrain, true);
  }

  public static Command figureEightCommand(TestTrajectories testTrajectories, Drivetrain Drivetrain) {
    return SwerveControllerCommand(testTrajectories.figureEight(0.5), Drivetrain, true);
  }

  public static Command curveLeftCommand(TestTrajectories testTrajectories, Drivetrain Drivetrain) {
    return SwerveControllerCommand(testTrajectories.simpleCurve(1.0, 1.0), Drivetrain, true);
  }

  public static Command curveRightCommand(TestTrajectories testTrajectories, Drivetrain Drivetrain) {
    return SwerveControllerCommand(testTrajectories.simpleCurve(1.0, -1.0), Drivetrain, true);
  }

  public static Command doNothingCommand(Drivetrain Drivetrain) {
      return new InstantCommand(() -> Drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
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
  public static void addTestTrajectoriesToChooser(SendableChooser<Command> chooser, double maxVelocity, double maxAcceleration, Drivetrain Drivetrain, boolean isSwerve) {

    TestTrajectories tt = new TestTrajectories(maxVelocity, maxAcceleration, Drivetrain, isSwerve);

    chooser.addOption("Figure 8", figureEightCommand(tt, Drivetrain));
    chooser.addOption("Go Forward 1m", straightForward1mCommand(tt, Drivetrain));
    chooser.addOption("Go Forward 2m", straightForward2mCommand(tt, Drivetrain));
    chooser.addOption("Go Back 1m", straightBack1mCommand(tt, Drivetrain));
    chooser.addOption("Curve Left", curveLeftCommand(tt, Drivetrain));
    chooser.addOption("Curve Right", curveRightCommand(tt, Drivetrain));
    if (isSwerve) {
      chooser.addOption("Go Sideways 1m", sidewaysLeft1mCommand(tt, Drivetrain));
    }
    chooser.setDefaultOption("Do Nothing", doNothingCommand(Drivetrain));
  }
}
