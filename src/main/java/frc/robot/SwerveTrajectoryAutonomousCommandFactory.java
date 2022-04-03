// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.StartPoseConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DriveChimpMode;
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
  private static TestTrajectories m_tt;
  private static LimelightSubsystem m_limelight;
  private static Robot m_robot;

  private static int m_shootRPM = 3000;


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
    m_tt = new TestTrajectories(maxVelocity, maxAcceleration, m_drivetrain, true);
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



  /**
   * Creates a command that makes the robot rudely replace one of the opposing team's cargo with the robot's team's
   * @param startPos the beginning position
   * @param ejectPos the position where the robot ejects its currently contained cargo
   * @param oppCargo the position of the target opposition cargo
   * @return
   */
  public Command cargoReplaceCommand(Pose2d startPos, Translation2d ejectPos, Translation2d oppCargo) {

    m_drivetrain.resetOdometry(startPos);

    // get the angle between the pose of the robot and the location of the cargo
    Rotation2d cargoAngle = new Rotation2d(getTranslationsAngleDouble(ejectPos, oppCargo));

    Trajectory moveToEjectPos = TrajectoryGenerator.generateTrajectory(startPos, List.of(),
        new Pose2d(ejectPos, cargoAngle), m_tt.getTrajectoryConfig());
    
    // pre-made trajectory that replaces red cargo 3
    PathPlannerTrajectory ball3trajectory = PathPlanner.loadPath("replaceCargo", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

    // pre-made trajectory that replaces red cargo 2 (for stealing jackson's cargo)
    PathPlannerTrajectory ball2trajectory = PathPlanner.loadPath("replaceCargo2", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

    Trajectory usedTrajectory = ball2trajectory;

    return new SequentialCommandGroup(

      SwerveControllerCommand(usedTrajectory, true),

      new IntakeReverseCommand(m_intake, m_cargo).withTimeout(4)

    );
  }

  /**
   * Shoots two cargo into our goal, and pushes one enemy cargo into our hangar
   */
  public Command twoBallEnemyOne() {

    PathPlannerTrajectory traject1 = PathPlanner.loadPath("2plus1ball_part1", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);
    PathPlannerTrajectory traject2 = PathPlanner.loadPath("2plus1ball_part2", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);
    PathPlannerTrajectory traject3 = PathPlanner.loadPath("2plus1ball_part3", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

    m_drivetrain.resetOdometry(traject1.getInitialPose());

    return new SequentialCommandGroup(

      new ParallelRaceGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        PPSwerveControlCommand(traject1, true)
      ),
      new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),

      new ParallelRaceGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        PPSwerveControlCommand(traject2, true)
      ),

      new IntakeReverseCommand(m_intake, m_cargo).withTimeout(5),

      PPSwerveControlCommand(traject3, true)
    );
  }

  // testing trajectories created by PathPlanner
  public Command changeHeading() {

    PathPlannerTrajectory path = PathPlanner.loadPath("test_changeheading", 1.5, 0.75);

    return PPSwerveControlCommand(path, true).beforeStarting(new InstantCommand(() ->m_drivetrain.resetOdometry(path.getInitialPose())));
  }

  public Command curve() {

    PathPlannerTrajectory path = PathPlanner.loadPath("test_curve", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

    return PPSwerveControlCommand(path, true).beforeStarting(new InstantCommand(() ->m_drivetrain.resetOdometry(path.getInitialPose())));
  }

  public Command straightLine() {

    PathPlannerTrajectory path = PathPlanner.loadPath("test_straightline", 1.5, 0.75);

    return PPSwerveControlCommand(path, true).beforeStarting(new InstantCommand(() ->m_drivetrain.resetOdometry(path.getInitialPose())));
  }

  public Command rightSideFiveBall() {

    PathPlannerTrajectory path1 = PathPlanner.loadPath("5ball_part1", 2.5, 2.0);

    PathPlannerTrajectory path2 = PathPlanner.loadPath("5ball_part2", 3.0, 2.5);

    PathPlannerTrajectory path3 = PathPlanner.loadPath("5ball_part3", 3.0, 2.5);

    PathPlannerTrajectory path4 = PathPlanner.loadPath("5ball_part4", 3.5, 3.5);


    return new SequentialCommandGroup( 
      new InstantCommand(() ->m_drivetrain.resetOdometry(path1.getInitialPose())),
      new ParallelCommandGroup(
        new IntakeDeployCommand(m_intake, m_cargo).until(() -> (m_cargo.cargoInLowerBelts() && m_cargo.cargoInUpperBelts())),
        PPSwerveControlCommand(path1, true)
      ),
      new ParallelRaceGroup(
         new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, m_limelight),
         //new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
         new ShootWithSetRPMAndHoodAngle(2900, 30, m_cargo, m_shooter, m_hood, m_robot)
      ),
      new ParallelRaceGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        PPSwerveControlCommand(path2, true)
      ),
      new ParallelRaceGroup(
        new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, m_limelight),
        new ShootWithSetRPMAndHoodAngle(3200, 30.5, m_cargo, m_shooter, m_hood, m_robot)
        //new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
      ),
      new ParallelRaceGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        new SequentialCommandGroup(
          PPSwerveControlCommand(path3, true),
          new WaitCommand(0.3)
        )
      ),
      new InstantCommand(() -> m_hood.setAngleDegrees(31)),
      PPSwerveControlCommand(path4, true),
      new ParallelRaceGroup(
        new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, m_limelight),
        new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
      )
    );
  }


  public Command leftSide2plus1() {

    PathPlannerTrajectory path1 = PathPlanner.loadPath("2plus1ball_part1", 2.0, 1.5);

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

  // private method that changes the rotation of a Pose2d (because that isn't already included in the class for some reason)
  private static Pose2d setRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getX(), pose.getY(), rotation);
  }

  // private method that gets the angle between two Translation2ds
  private static Rotation2d getTranslationsAngle(Translation2d pose1, Translation2d pose2) {
    Vector2d vector1 = new Vector2d(pose1.getX(), pose1.getY());
    Vector2d vector2 = new Vector2d(pose2.getX(), pose2.getY());
    double dotProduct = vector1.dot(vector2);
    double magnitude = vector1.magnitude() * vector2.magnitude();

    return new Rotation2d( Math.acos(dotProduct/magnitude) );
  }

  // private method that gets the angle between two Translation2ds
  private static double getTranslationsAngleDouble(Translation2d pose1, Translation2d pose2) {
    Vector2d vector1 = new Vector2d(pose1.getX(), pose1.getY());
    Vector2d vector2 = new Vector2d(pose2.getX(), pose2.getY());
    double dotProduct = vector1.dot(vector2);
    double magnitude = vector1.magnitude() * vector2.magnitude();

    return Math.acos(dotProduct/magnitude);
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
