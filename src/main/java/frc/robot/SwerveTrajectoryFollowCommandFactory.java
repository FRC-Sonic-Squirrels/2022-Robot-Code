// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.midi.Track;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.HubCentricConstants;
import frc.robot.commands.CargoMoveToUpperBeltsCommand;
import frc.robot.commands.IntakeDeployCommand;
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

  public static Command testAutonCommand(TestTrajectories testTrajectories, Drivetrain drivetrain){
    Trajectory path1 = testTrajectories.simpleCurve(10, -20);
    Trajectory path2 = testTrajectories.straightForward(10);
    Trajectory path3 = testTrajectories.simpleCurve(-10, 20);
    Trajectory path4 = testTrajectories.straightForward(-10);

    Command finalCOmmand = new SequentialCommandGroup(
      SwerveControllerCommand(path1, drivetrain, false),
      SwerveControllerCommand(path2, drivetrain, false),
      SwerveControllerCommand(path3, drivetrain, false),
      SwerveControllerCommand(path4, drivetrain, false)
    );

    return finalCOmmand;

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
  public static void addTestTrajectoriesToChooser(SendableChooser<Command> chooser, double maxVelocity,
      double maxAcceleration, Drivetrain drivetrain, boolean isSwerve) {

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

  // command for shooting directly from the tarmac in autonomous mode
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

  /**
   * command for autonomously shooting and then moving to the next set cargo coordinates.
   * this will most likely be used for the autonomous phase of the competition
   * @param testTrajectories the TestTrajectories class
   * @param drivetrain the drivetrain class
   * @param shooter the shooter class
   * @param cargo the cargo class
   * @param intake the intake class
   * @param cargoPose2d the position of the wanted cargo (coordinates are in units of inches)
   * @return a set of actions with the robot shooting its current cargo, then moving and picking up the next cargo
   */

  public static Command shootAndMoveToCargoCommand(TestTrajectories testTrajectories, Drivetrain drivetrain,
      ShooterSubsystem shooter, CargoSubsystem cargo, IntakeSubsystem intake, Pose2d cargoPose2d, Pose2d shootPos) {

    // assuming the angle is set rather than added: angle = arctangent ((robotX - hubX) / (robotY - hubY))
    // hub/center coordinates: (324, 162)
    // The robot will most likely start at a 0 degree angle
    Rotation2d shootAngle = new Rotation2d( Math.atan((shootPos.getX() - HubCentricConstants.HUB_CENTER.x))
                                                    / (shootPos.getY() - HubCentricConstants.HUB_CENTER.y));

    Pose2d startPos = drivetrain.getPose();
    // positioned to be about to load the cargo (this means facing the cargo as well)
    Pose2d midPos = new Pose2d(startPos.getX() - 1, startPos.getY(), new Rotation2d(0)); 
    Pose2d cargoPos = new Pose2d(Units.inchesToMeters(cargoPose2d.getX()), Units.inchesToMeters(cargoPose2d.getY()),
        new Rotation2d(0));


    double rpm = 0;

    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new CargoMoveToUpperBeltsCommand(cargo),
        new WaitUntilCommand(() -> cargo.cargoInUpperBelts()),
        new InstantCommand(() -> shooter.setFlywheelRPM(rpm)),
        new WaitUntilCommand(() -> shooter.isAtDesiredRPM()),
        // TODO: have the robot face the hub, then shoot
        SwerveControllerCommand(testTrajectories.driveToPose(new Pose2d(shootPos.getX(), shootPos.getY(), shootAngle),
            drivetrain.getPose()), drivetrain, true)
      ),
      new ShootOneCargoCommand(cargo, shooter, intake),
      SwerveControllerCommand(testTrajectories.driveToPose(startPos, midPos), drivetrain, true),
      new IntakeDeployCommand(intake, cargo),
      new WaitUntilCommand(() -> intake.intakeAtDesiredRPM()),
      SwerveControllerCommand(testTrajectories.driveToPose(midPos, cargoPos), drivetrain, true)
    );
  }

  public static Command moveToPoseCommand(TestTrajectories testTrajectories, Drivetrain drivetrain, Pose2d target) {

    Pose2d current = drivetrain.getPose();
    target = new Pose2d(Units.inchesToMeters(target.getX()), Units.inchesToMeters(target.getY()), target.getRotation());
    return SwerveControllerCommand(testTrajectories.driveToPose(current, target), drivetrain, true);
  }
}
