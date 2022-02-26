// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.HubCentricConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.CargoMoveToUpperBeltsCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.ShootCargoCommand;
import frc.robot.commands.ShootOneCargoCommand;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SwerveTrajectoryAutonomousCommandFactory {

  public static void addAutonTrajectoriesToChooser(SendableChooser<Command> chooser, double maxVelocity,
      double maxAcceleration, Pose2d start, Drivetrain drivetrain, boolean isSwerve, ShooterSubsystem shooter,
      CargoSubsystem cargo, IntakeSubsystem intake, Robot robot) {

    TestTrajectories tt = new TestTrajectories(maxVelocity, maxAcceleration, drivetrain, isSwerve);
    
    //TODO: have the mid poses be different than the cargo poses and find the shoot pose
    chooser.addOption("4 blue auton 123", fourBallAutonCommand( start,
        new Pose2d(FieldConstants.BLUE_CARGO_1, new Rotation2d()), new Pose2d(FieldConstants.BLUE_CARGO_1, new Rotation2d()), 
        null,
        new Pose2d(FieldConstants.BLUE_CARGO_2, new Rotation2d()), new Pose2d(FieldConstants.BLUE_CARGO_2, new Rotation2d()),
        new Pose2d(FieldConstants.BLUE_CARGO_3, new Rotation2d()), new Pose2d(FieldConstants.BLUE_CARGO_3, new Rotation2d()),
        tt, drivetrain, shooter, cargo, intake, robot));
    
    chooser.addOption("4 blue auton 237", fourBallAutonCommand( start,
        null, null,
        null,
        null, null,
        null, null,
        tt, drivetrain, shooter, cargo, intake, robot));

    chooser.addOption("4 blue auton 372", fourBallAutonCommand( start,
        null, null,
        null,
        null, null,
        null, null,
        tt, drivetrain, shooter, cargo, intake, robot));
  }

   /**
   * command for autonomously shooting and then moving to the next set of cargo coordinates.
   * @param cargoPos the position of the wanted cargo 
   * @param shootPos the position of the robot when about to shoot cargo 
   * @param midPos the position right in front of the wanted cargo 
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


  
  // command that lets the robot intake 1 cargo without shooting it (parameters have already been converted to meters)
  public static Command intakeCargoCommand(TestTrajectories testTrajectories, Drivetrain drivetrain,
      CargoSubsystem cargo, IntakeSubsystem intake, Pose2d targetPos, Pose2d midPos) {
    
    // precondition: the robot must have at least one free belt
    if (cargo.cargoInLowerBelts() && cargo.cargoInUpperBelts()) {
      return null;
    }

    Trajectory startToMid = testTrajectories.driveToPose(drivetrain.getPose(), midPos);
    //startToMid.transformBy(new Transform2d(drivetrain.getPose(), midPos));
    Trajectory midToTarget = testTrajectories.driveToPose(midPos, targetPos);
    //midToTarget.transformBy(new Transform2d(midPos, targetPos));

    return new SequentialCommandGroup(
        // 1. deploy intake then move to the front of a cargo. if there is a stored cargo move it to
        // the upper belts
        new ParallelCommandGroup(new CargoMoveToUpperBeltsCommand(cargo),
            SwerveControllerCommand(startToMid, drivetrain, true),
            new IntakeDeployCommand(intake, cargo),
            new WaitUntilCommand(() -> intake.intakeAtDesiredRPM()),
            new WaitUntilCommand(() -> !cargo.cargoInLowerBelts())),

        // 2. collect the cargo, then wait until it is fully in the lower belts
        SwerveControllerCommand(midToTarget, drivetrain, true),
        new WaitUntilCommand(() -> cargo.cargoInLowerBelts()),

        // 3. retract and deactivate the intake
        new ParallelCommandGroup(new InstantCommand(() -> intake.retractIntake()),
            new InstantCommand(() -> intake.stop())));

  }


  /**
   * command for autonomously shooting two cargo then collecting the next two cargo by coordinates.
   * (i apologize for this method having 13 parameters)
   * @param startPos the robots position at the beginning of the match (a constant)
   * @param initCargoPos the position of the pre-shoot cargo (meters)
   * @param initMidPos the position before collecting the pre-shoot cargo (meters)
   * @param shootPos the position of the robot when about to shoot cargo (in meters)
   * @param cargoPos1 the position of the wanted cargo (in meters)
   * @param midPos1 the position right in front of the wanted cargo (in meters)
   * @param cargoPos2 the position of the second wanted cargo (meters)
   * @param midPos2 the position right in front of second wanted cargo (meters)
   * @return a set of actions with the robot shooting its current 2 cargo, then moving and picking up the next 2 cargo
   */
  public static Command fourBallAutonCommand(Pose2d startPos, Pose2d initCargoPos, Pose2d initMidPos, Pose2d shootPos, Pose2d cargoPos1,
      Pose2d midPos1, Pose2d cargoPos2, Pose2d midPos2, TestTrajectories testTrajectories, Drivetrain drivetrain,
      ShooterSubsystem shooter, CargoSubsystem cargo, IntakeSubsystem intake, Robot robot) {

    drivetrain.setPose(startPos, startPos.getRotation());

    // give shootAngle its rotation, then change units
    Rotation2d shootAngle = new Rotation2d( Math.atan2(shootPos.getY() - HubCentricConstants.HUB_CENTER.y,
        shootPos.getX() - HubCentricConstants.HUB_CENTER.x));
    //shootPos = inchesToMeters(shootPos);
    shootPos = setRotation(shootPos, shootAngle);
    
    // change units of mid poses
    //initMidPos = inchesToMeters(initMidPos);
    //midPos1 = inchesToMeters(midPos1);
    //midPos2 = inchesToMeters(midPos2);

    // change units of cargo poses
    //initCargoPos = inchesToMeters(initCargoPos);
    //cargoPos1 = inchesToMeters(cargoPos1);
    //cargoPos2 = inchesToMeters(cargoPos2);

    // find the angle the robot needs to be at to pick up the three different cargo, then set the midPoses to those angles
    Rotation2d initAngle = getPosesAngle(initMidPos, initCargoPos);
    initMidPos = setRotation(initMidPos, initAngle);

    Rotation2d angle1 = getPosesAngle(midPos1, cargoPos1);
    midPos1 = setRotation(midPos1, angle1);

    Rotation2d angle2 = getPosesAngle(midPos2, cargoPos2);
    midPos2 = setRotation(midPos2, angle2);

    // instantiate all the trajectories
    //Trajectory start_to_initMidPos = testTrajectories.driveToPose(drivetrain.getPose(), shootPos);
    //Trajectory initMidPos_to_initCargoPos = testTrajectories.driveToPose(shootPos, midPos1);
    Trajectory initCargoPos_to_shootPos = testTrajectories.driveToPose(initCargoPos, shootPos);
    Trajectory shootPos_to_midPos1 = testTrajectories.driveToPose(shootPos, midPos1);
    Trajectory midPos1_to_cargoPos1 = testTrajectories.driveToPose(midPos1, cargoPos1);
    Trajectory cargoPos1_to_midPos2 = testTrajectories.driveToPose(cargoPos1, midPos2);
    Trajectory midPos2_to_cargoPos2 = testTrajectories.driveToPose(midPos2, cargoPos2);
    

    return new SequentialCommandGroup(
      // 0. take in one cargo, in order to have two cargo
      intakeCargoCommand(testTrajectories, drivetrain, cargo, intake, initCargoPos, initMidPos),

      // 1. set up flywheel, then move to the shooting location
      new ParallelCommandGroup(
        new InstantCommand(() -> shooter.setFlywheelRPM(ShooterConstants.m_activated)),
        new WaitUntilCommand(() -> shooter.isAtDesiredRPM()),
        SwerveControllerCommand(initCargoPos_to_shootPos, drivetrain, true)
      ),
      
      // 2. shoot both cargo
      new ShootCargoCommand(cargo, shooter, intake, robot),
      
      // 3. slow down flywheel, deploy the intake, then move in front of the first cargo
      new ParallelCommandGroup(
        new InstantCommand(() -> shooter.setFlywheelRPM(ShooterConstants.m_idle)),
        SwerveControllerCommand(shootPos_to_midPos1, drivetrain, true),
        new IntakeDeployCommand(intake, cargo),
        new WaitUntilCommand(() -> intake.intakeAtDesiredRPM())
      ),

      // 4. collect the first cargo, then wait until it is fully in the upper belts
      SwerveControllerCommand(midPos1_to_cargoPos1, drivetrain, true),
      new WaitUntilCommand(() -> cargo.cargoInUpperBelts()),
      
      // 5. move from where the first cargo was, to the second mid position
      SwerveControllerCommand(cargoPos1_to_midPos2, drivetrain, true),

      // 6. collect the second cargo, then wait until it is fully in the lower belts
      SwerveControllerCommand(midPos2_to_cargoPos2, drivetrain, true),
      new WaitUntilCommand(() -> cargo.cargoInLowerBelts()),
      
      // 7. retract and deactivate the intake
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.retractIntake()),
        new InstantCommand(() -> intake.stop())
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

  // private method that changes the rotation of a Pose2d (because that isn't already included in the class for some reason)
  private static Pose2d setRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getX(), pose.getY(), rotation);
  }

  // private method that gets the angle between two Pose2ds
  private static Rotation2d getPosesAngle(Pose2d pose1, Pose2d pose2) {
    Vector2d vector1 = new Vector2d(pose1.getX(), pose1.getY());
    Vector2d vector2 = new Vector2d(pose2.getX(), pose2.getY());
    double dotProduct = vector1.dot(vector2);
    double magnitude = vector1.magnitude() * vector2.magnitude();

    return new Rotation2d( Math.acos(dotProduct/magnitude) );
  }


}
