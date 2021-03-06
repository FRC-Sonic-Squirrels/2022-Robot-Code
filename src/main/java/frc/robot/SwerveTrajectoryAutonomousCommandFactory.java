// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team2930.lib.util.SwerveTestTrajectories;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.StartPoseConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DriveFieldCentricAimCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.LimelightAutoShoot;
import frc.robot.commands.ShootWithSetRPMAndHoodAngle;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SwerveTrajectoryAutonomousCommandFactory {

  private static ShooterSubsystem m_shooter;
  private static Drivetrain m_drivetrain;
  private static CargoSubsystem m_cargo;
  private static IntakeSubsystem m_intake;
  private static HoodSubsystem m_hood;
  private static SwerveTestTrajectories m_tt;
  private static LimelightSubsystem m_limelight;
  private static Robot m_robot;


  public SwerveTrajectoryAutonomousCommandFactory(Drivetrain drivetrain, ShooterSubsystem shooter,
      CargoSubsystem cargo, IntakeSubsystem intake, HoodSubsystem hood, LimelightSubsystem limelight,
      Robot robot, double maxVelocity, double maxAcceleration) {

    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_cargo = cargo;
    m_intake = intake;
    m_hood = hood;
    m_limelight = limelight;
    m_robot = robot;

    m_tt = new SwerveTestTrajectories(maxVelocity, maxAcceleration,
        Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, drivetrain.kinematics());
  }


  /**
   * twoBallAutoShoot2Push -  drive, pickup cargo, drive back, shoot two, then push opponent cargo out of the way
   */
  public Command twoBallAutoShoot2push() {

    Pose2d startPos = StartPoseConstants.BLUE_DEF_TOP;
    Translation2d cargoPos = FieldConstants.BLUE_CARGO_7;

    m_drivetrain.resetOdometry(startPos);

    Pose2d ourCargoPose = new Pose2d( cargoPos, new Rotation2d(3*Math.PI/4) );
    Pose2d opponentCargoPose = new Pose2d(Constants.FieldConstants.RED_CARGO_4, new Rotation2d(Math.PI/2));
    Pose2d backPose = new Pose2d(Constants.FieldConstants.BEHIND_RED_CARGO_4, new Rotation2d(Math.PI));

    Trajectory moveToCargoOne =  TrajectoryGenerator.generateTrajectory(startPos,
        List.of(), ourCargoPose, m_tt.getTrajectoryConfig());

    Trajectory moveToHub =  TrajectoryGenerator.generateTrajectory(ourCargoPose,
        List.of(), startPos, m_tt.getTrajectoryConfig());

    Trajectory moveToOpponentCargo =  TrajectoryGenerator.generateTrajectory(startPos,
        List.of(), opponentCargoPose, m_tt.getTrajectoryConfig());

    Trajectory rotateAndMove = TrajectoryGenerator.generateTrajectory(opponentCargoPose,
    List.of(), backPose, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(      

        new ParallelRaceGroup(
            SwerveControllerCommand(moveToCargoOne, true),
            new IntakeDeployCommand(m_intake, m_cargo)
        ),
        
        SwerveControllerCommand(moveToHub, true),

        // shoot
        new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),

        // pick up opponent ball
        new ParallelRaceGroup(
            new IntakeDeployCommand(m_intake, m_cargo),
            SwerveControllerCommand(moveToOpponentCargo, true)
        ),

        // turn and eject opponent ball towards our hanger zone
        SwerveControllerCommand(rotateAndMove, true),
        new IntakeReverseCommand(m_intake, m_cargo).withTimeout(2));
  }  

  /**
   * twoBallAutoShoot2 -  drive, pickup cargo, wait, drive back, shoot two
   */
  public Command twoBallAutoWaitShoot2() {

    Pose2d startPos = StartPoseConstants.BLUE_TOP;
    Pose2d cargoPos = new Pose2d(FieldConstants.BLUE_CARGO_7, new Rotation2d(Math.PI));
    Pose2d shootPos = StartPoseConstants.BLUE_DEF_TOP;

    m_drivetrain.resetOdometry(startPos);

    Trajectory moveToCargoOne =  TrajectoryGenerator.generateTrajectory(startPos,
        List.of(), cargoPos, m_tt.getTrajectoryConfig());

    Trajectory moveToHub =  TrajectoryGenerator.generateTrajectory(cargoPos,
        List.of(), shootPos, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrain.resetOdometry(startPos)),
      new ParallelRaceGroup(
        SwerveControllerCommand(moveToCargoOne, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),
      //new WaitCommand(waitTime),
      new InstantCommand(() -> m_shooter.setFlywheelRPM(2750), m_shooter),
      SwerveControllerCommand(moveToHub, true),
      new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
        .withTimeout(6)
    );
  }  

  public Command SundomeRightSideShootAndMove(){
    Pose2d startPos = Constants.StartPoseConstants.BLUE_DEF_BOTTOM;
    Translation2d cargoPos = Constants.FieldConstants.BLUE_CARGO_2;

    Trajectory start_to_cargo = TrajectoryGenerator.generateTrajectory(
      startPos, List.of(), 
      new Pose2d(cargoPos, startPos.getRotation()), m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(
      new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
        .withTimeout(4),

      SwerveControllerCommand(start_to_cargo, true)
    );
  }


  public Command curve() {

    PathPlannerTrajectory path = PathPlanner.loadPath("test_curve", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

    return PPSwerveControlCommand(path, true).beforeStarting(new InstantCommand(() ->m_drivetrain.resetOdometry(path.getInitialPose())));
  }

  public Command straightLine() {

    PathPlannerTrajectory path = PathPlanner.loadPath("test_straightline", 1.5, 0.75);

    return PPSwerveControlCommand(path, true).beforeStarting(new InstantCommand(() ->m_drivetrain.resetOdometry(path.getInitialPose())));
  }

  public Command middleShootFenderAndLeave(){

    PathPlannerTrajectory path1 = PathPlanner.loadPath("1ball_complementary_part1", 1.5, 0.75);
    PathPlannerTrajectory path2 = PathPlanner.loadPath("1ball_complementary_part2", 1.5, 0.75);

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrain.resetOdometry(path1.getInitialPose())),
      new WaitCommand(4),
      PPSwerveControlCommand(path1, true),
      new ParallelRaceGroup(
        new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, m_limelight),
        new ShootWithSetRPMAndHoodAngle(3000, 30.0, m_cargo, m_shooter, m_hood, m_robot)
      ),
      PPSwerveControlCommand(path2, true)
    );
  }

  /**
   * rightSideFiveBall() - competition auton for right side, 5 ball
   *
   * @return
   */
  public Command rightSideFiveBall() {

    PathPlannerTrajectory path1 = PathPlanner.loadPath("5ball_part1", 3.0, 2.2);

    PathPlannerTrajectory path2 = PathPlanner.loadPath("5ball_part2", 3.0, 2.5);

    PathPlannerTrajectory path3 = PathPlanner.loadPath("5ball_part3", 4.5, 3.5);

    PathPlannerTrajectory path4 = PathPlanner.loadPath("5ball_part4", 4.5, 3.5);

    return new SequentialCommandGroup( 
      new InstantCommand(() ->m_drivetrain.resetOdometry(path1.getInitialPose())),
      new InstantCommand(() -> m_hood.setAngleDegrees(30)),
      new InstantCommand(() -> m_shooter.setFlywheelRPM(2900)),
      new ParallelCommandGroup(
        new IntakeDeployCommand(m_intake, m_cargo).until(() -> (m_cargo.cargoInLowerBelts() && m_cargo.cargoInUpperBelts())).withTimeout(3),
        PPSwerveControlCommand(path1, true)
      ),
      new ShootWithSetRPMAndHoodAngle(2900, 30, m_cargo, m_shooter, m_hood, m_robot),
      // new ParallelRaceGroup(
      //    new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, m_limelight),
      //    //new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
      //    new ShootWithSetRPMAndHoodAngle(2900, 30, m_cargo, m_shooter, m_hood, m_robot)
      // ),
      new ParallelRaceGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        PPSwerveControlCommand(path2, true)
      ),
      new ShootWithSetRPMAndHoodAngle(3350, 32.5, m_cargo, m_shooter, m_hood, m_robot),
      //new ParallelRaceGroup(
        //new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, m_limelight),
        //new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
      //),
      new ParallelCommandGroup(
        // run the intake while we drive forward, but retract after we start driving
        // away from loading zone. vx is robot centric x velocity.
        new IntakeDeployCommand(m_intake, m_cargo)
          .until(() -> (m_drivetrain.getChassisSpeeds().vxMetersPerSecond < -0.5)),
        new SequentialCommandGroup(
          PPSwerveControlCommand(path3, true),
          new WaitCommand(0.4),
          new InstantCommand(() -> m_hood.setAngleDegrees(30)),
          new InstantCommand(() -> m_shooter.setFlywheelRPM(3000)),
          PPSwerveControlCommand(path4, true)
        )
      ),
      new ParallelRaceGroup(
        new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, m_limelight),
        //new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
        new ShootWithSetRPMAndHoodAngle(3000, 30.0, m_cargo, m_shooter, m_hood, m_robot)
      ),
      new InstantCommand(() -> m_hood.setMinAngle()),
      new InstantCommand(() -> m_shooter.setFlywheelRPM(Constants.ShooterConstants.IDLE), m_shooter)
    );
  }


  /**
   * leftSide2plus1() - left side, score 2 and hide one opponent cargo.
   * 
   * @return
   */
  public Command leftSide2plus1() {

    PathPlannerTrajectory path1 = PathPlanner.loadPath("2plus1ball_part1", 3.0, 1.5);

    PathPlannerTrajectory path1b = PathPlanner.loadPath("2plus1ball_part1b", 2.5, 2.0);

    PathPlannerTrajectory path2 = PathPlanner.loadPath("2plus1ball_part2", 2.5, 2.0);

    PathPlannerTrajectory path3 = PathPlanner.loadPath("2plus1ball_part3", 3.0, 2.0);

    return new SequentialCommandGroup( 
      new InstantCommand(() ->m_drivetrain.resetOdometry(path1.getInitialPose())),
      new ParallelCommandGroup(
        new IntakeDeployCommand(m_intake, m_cargo).until(() -> (m_cargo.cargoInLowerBelts() && m_cargo.cargoInUpperBelts())),
        PPSwerveControlCommand(path1, true)
      ),
      new InstantCommand(() -> m_shooter.setFlywheelRPM(2800)),
      new InstantCommand(() -> m_hood.setAngleDegrees(29)),
      PPSwerveControlCommand(path1b, true),
      new ParallelRaceGroup(
         new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, m_limelight),
         //new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
         new ShootWithSetRPMAndHoodAngle(2800, 29, m_cargo, m_shooter, m_hood, m_robot)
      ),
      new ParallelCommandGroup(
        new IntakeDeployCommand(m_intake, m_cargo).until(() -> m_cargo.cargoInLowerBelts()).withTimeout(3.0),
        PPSwerveControlCommand(path2, true)
      ),
      new IntakeReverseCommand(m_intake, m_cargo).withTimeout(2.0),
      PPSwerveControlCommand(path3, true)
      );
  }
  public Command testShootBall() {
    return new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot);
  }


  /**
   * Create a swerve trajectory follow command. If stopAtEnd is set to true, robot will come to full
   * stop at the end of the command.
   * 
   * @param trajectory
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

  /**
   * Create a swerve trajectory follow command. If stopAtEnd is set to true, robot will come to full stop when done.
   * 
   * @param trajectory
   * @param stopAtEnd
   * @return
   */
  public static Command PPSwerveControlCommand(PathPlannerTrajectory trajectory, boolean stopAtEnd){

    var thetaController =
        new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController,
            AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Command swerveControllerCommand = new PPSwerveControllerCommand(trajectory,
     m_drivetrain::getPose,
     m_drivetrain.kinematics(), 
     new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
     new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
     thetaController,
     m_drivetrain::setModuleStates, 
     m_drivetrain);

     if (stopAtEnd) {
      // Stop at the end. A good safe default, but not desireable if running two paths back to back
      swerveControllerCommand = swerveControllerCommand
          .andThen(() -> m_drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
    }
    return swerveControllerCommand;
  }

}
