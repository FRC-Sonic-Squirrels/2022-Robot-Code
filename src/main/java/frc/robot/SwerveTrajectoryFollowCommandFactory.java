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
import frc.robot.Constants.ShooterConstants;
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
   * command for autonomously shooting and then moving to the next set of cargo coordinates.
   * @param cargoPos the position of the wanted cargo (in inches)
   * @param shootPos the position of the robot when about to shoot cargo (in inches)
   * @param midPos the position right in front of the wanted cargo (in inches)
   * @return a set of actions with the robot shooting its current cargo, then moving and picking up the next cargo
   */
  public static Command shootAndMoveToCargoCommand(Pose2d cargoPos, Pose2d shootPos, Pose2d midPos,
      TestTrajectories testTrajectories, Drivetrain drivetrain, ShooterSubsystem shooter, CargoSubsystem cargo, IntakeSubsystem intake) {

    // hub/center coordinates: (324, 162)
    // The robot will most likely start at a 0 degree angle
    Rotation2d shootAngle = new Rotation2d( Math.atan((shootPos.getX() - HubCentricConstants.HUB_CENTER.x))
                                                    / (shootPos.getY() - HubCentricConstants.HUB_CENTER.y));

    // positioned to be about to load the cargo (this means facing the cargo as well) 
    cargoPos = new Pose2d(Units.inchesToMeters(cargoPos.getX()), Units.inchesToMeters(cargoPos.getY()), new Rotation2d(0));

    Trajectory startToShoot = testTrajectories.driveToPose(drivetrain.getPose(), new Pose2d(shootPos.getX(), shootPos.getY(), shootAngle));

    //startToShoot.transformBy(new Transform2d(new Pose2d(shootPos.getX(), shootPos.getY(), shootAngle), drivetrain.getPose()));

    return new SequentialCommandGroup(
      // 1. move cargo to upper belts, set up flywheel, then move to the shooting location
      new ParallelCommandGroup(
        new CargoMoveToUpperBeltsCommand(cargo),
        new WaitUntilCommand(() -> cargo.cargoInUpperBelts()),
        new InstantCommand(() -> shooter.setFlywheelRPM(ShooterConstants.m_activated)),
        new WaitUntilCommand(() -> shooter.isAtDesiredRPM()),
        SwerveControllerCommand(startToShoot, drivetrain, true)
      ),
      
      // 2. shoot the cargo
      new ShootOneCargoCommand(cargo, shooter, intake),
      
      // 3. slow down flywheel, deploy the intake, then move in front of the next cargo
      new ParallelCommandGroup(
        new InstantCommand(() -> shooter.setFlywheelRPM(ShooterConstants.m_idle)),
        SwerveControllerCommand(testTrajectories.driveToPose(drivetrain.getPose(), midPos), drivetrain, true),
        new IntakeDeployCommand(intake, cargo),
        new WaitUntilCommand(() -> intake.intakeAtDesiredRPM())
      ),
      
      // 4. collect the cargo, then wait until it is fully in the lower belts
      SwerveControllerCommand(testTrajectories.driveToPose(midPos, cargoPos), drivetrain, true),
      new WaitUntilCommand(() -> cargo.cargoInLowerBelts()),

      // 5. retract and deactivate the intake
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.retractIntake()),
        new InstantCommand(() -> intake.stop())
      )
    );

  }

  // simpler command that moves the robot from its current position to another set of coordinates
  public static Command moveToPoseCommand(TestTrajectories testTrajectories, Drivetrain drivetrain, Pose2d target) {

    Pose2d current = drivetrain.getPose();
    
    target = inchesToMeters(target);

    return SwerveControllerCommand(testTrajectories.driveToPose(current, target), drivetrain, true);
  }


  // command that lets the robot intake 1 cargo without shooting it
  public static Command intakeCargoCommand(TestTrajectories testTrajectories, Drivetrain drivetrain, CargoSubsystem cargo,
      IntakeSubsystem intake, Pose2d targetPos, Pose2d midPos) {
    
    // precondition: the robot must have at least one free belt
    if (cargo.cargoInLowerBelts() && cargo.cargoInUpperBelts()) {
      return null;
    }

    Trajectory startToMid = testTrajectories.driveToPose(drivetrain.getPose(), midPos);
    //startToMid.transformBy(new Transform2d(drivetrain.getPose(), midPos));
    Trajectory midToTarget = testTrajectories.driveToPose(midPos, targetPos);
    //midToTarget.transformBy(new Transform2d(midPos, targetPos));

    return new SequentialCommandGroup(
      // 1. deploy intake then move to the front of a cargo. if there is a stored cargo move it to the upper belts
      new ParallelCommandGroup(
        new CargoMoveToUpperBeltsCommand(cargo),
        SwerveControllerCommand(startToMid, drivetrain, true),
        new IntakeDeployCommand(intake, cargo),
        new WaitUntilCommand(() -> intake.intakeAtDesiredRPM()),
        new WaitUntilCommand(() -> ! cargo.cargoInLowerBelts())
      ),
      
      // 2. collect the cargo, then wait until it is fully in the lower belts
      SwerveControllerCommand(midToTarget, drivetrain, true),
      new WaitUntilCommand(() -> cargo.cargoInLowerBelts()),

      // 3. retract and deactivate the intake
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.retractIntake()),
        new InstantCommand(() -> intake.stop())
      )
    );

  }


    /**
   * command for autonomously shooting two cargo then collecting the next two cargo by coordinates.
   * @param cargoPos1 the position of the wanted cargo (in inches)
   * @param midPos1 the position of the robot when about to shoot cargo (in inches)
   * @param shootPos the position right in front of the wanted cargo (in inches)
   * @param cargoPos2 the position of the second wanted cargo (inches)
   * @param midPos2 the position right in front of second wanted cargo (inches)
   * @return a set of actions with the robot shooting its current 2 cargo, then moving and picking up the next 2 cargo
   */
  public static Command doubleShootAndMoveToCargoCommand(Pose2d cargoPos1, Pose2d midPos1, Pose2d shootPos, Pose2d cargoPos2,
      Pose2d midPos2, TestTrajectories testTrajectories, Drivetrain drivetrain, ShooterSubsystem shooter,
      CargoSubsystem cargo, IntakeSubsystem intake) {

    // assuming the angle is set rather than added: angle = arctangent ((robotX - hubX) / (robotY - hubY))
    // hub/center coordinates: (324, 162)
    // The robot will most likely start at a 0 degree angle

    // give shootAngle its rotation, then change units
    Rotation2d shootAngle = new Rotation2d( Math.atan((shootPos.getX() - HubCentricConstants.HUB_CENTER.x))
        / (shootPos.getY() - HubCentricConstants.HUB_CENTER.y));
    shootPos = inchesToMeters(shootPos);
    shootPos = setRotation(shootPos, shootAngle);
    
    // change units of mid poses
    midPos1 = inchesToMeters(midPos1);
    midPos2 = inchesToMeters(midPos2);

    // change units of cargo poses
    cargoPos1 = inchesToMeters(cargoPos1);
    cargoPos2 = inchesToMeters(cargoPos2);

    Trajectory startToShoot = testTrajectories.driveToPose(drivetrain.getPose(), shootPos);

    return new SequentialCommandGroup(
      // 1. set up flywheel, then move to the shooting location
      new ParallelCommandGroup(
        new InstantCommand(() -> shooter.setFlywheelRPM(ShooterConstants.m_activated)),
        new WaitUntilCommand(() -> shooter.isAtDesiredRPM()),
        SwerveControllerCommand(startToShoot, drivetrain, true)
      ),
      
      // 2. shoot the cargo
      // FIXME: create a command that autonomously shoots both cargo
      new ShootOneCargoCommand(cargo, shooter, intake),
      
      // 3. slow down flywheel, deploy the intake, then move in front of the first cargo
      new ParallelCommandGroup(
        new InstantCommand(() -> shooter.setFlywheelRPM(ShooterConstants.m_idle)),
        SwerveControllerCommand(testTrajectories.driveToPose(drivetrain.getPose(), midPos1), drivetrain, true),
        new IntakeDeployCommand(intake, cargo),
        new WaitUntilCommand(() -> intake.intakeAtDesiredRPM())
      ),
      // 4. collect the first cargo, then wait until it is fully in the lower belts
      SwerveControllerCommand(testTrajectories.driveToPose(midPos1, cargoPos1), drivetrain, true),
      new WaitUntilCommand(() -> cargo.cargoInLowerBelts(),
      
      // TODO: add steps 5 and 6 to repeat steps 3 and 4 but with the second target cargo (excluding changing the flywheel/intake)
      // 5. move in front of the second cargo
      SwerveControllerCommand(testTrajectories.driveToPose(drivetrain.getPose(), midPos2), targetPos), drivetrain, true),
      

      // 7. retract and deactivate the intake
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.retractIntake()),
        new InstantCommand(() -> intake.stop())
      )
    );
  }


  // private method that will convert a pose2d in inches to meters
  private static Pose2d inchesToMeters(Pose2d pose) {
    return new Pose2d(Units.inchesToMeters(pose.getX()), Units.inchesToMeters(pose.getY()), pose.getRotation());
  }

  // private method that will change the rotation of a Pose2d
  private static Pose2d setRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getX(), pose.getY(), rotation);
  }
}
