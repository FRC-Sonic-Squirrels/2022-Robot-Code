// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team2930.lib.util.SwerveTestTrajectories;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
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
      CargoSubsystem cargo, IntakeSubsystem intake, HoodSubsystem hood,
      LimelightSubsystem limelight, Robot robot, double maxVelocity, double maxAcceleration) {

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

  public Command curve() {

    PathPlannerTrajectory path = PathPlanner.loadPath("test_curve", AutoConstants.maxVelocity,
        AutoConstants.maxAcceleration);

    return PPSwerveControlCommand(path, true).beforeStarting(
        new InstantCommand(() -> m_drivetrain.resetOdometry(path.getInitialPose())));
  }

  public Command straightLine() {

    PathPlannerTrajectory path = PathPlanner.loadPath("test_straightline", 1.5, 0.75);

    return PPSwerveControlCommand(path, true).beforeStarting(
        new InstantCommand(() -> m_drivetrain.resetOdometry(path.getInitialPose())));
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

    Pose2d startPose = getStartPoseForPath(path1);

    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrain.resetOdometry(startPose)),
        new InstantCommand(() -> m_hood.setAngleDegrees(30)),
        new InstantCommand(() -> m_shooter.setFlywheelRPM(2900)),
        new ParallelCommandGroup(new IntakeDeployCommand(m_intake, m_cargo)
            .until(() -> (m_cargo.cargoInLowerBelts() && m_cargo.cargoInUpperBelts()))
            .withTimeout(3), PPSwerveControlCommand(path1, true)),
        new ShootWithSetRPMAndHoodAngle(2900, 30, m_cargo, m_shooter, m_hood, m_robot),
        // new ParallelRaceGroup(
        // new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
        // m_limelight),
        // //new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
        // new ShootWithSetRPMAndHoodAngle(2900, 30, m_cargo, m_shooter, m_hood, m_robot)
        // ),
        new ParallelRaceGroup(new IntakeDeployCommand(m_intake, m_cargo),
            PPSwerveControlCommand(path2, true)),
        new ShootWithSetRPMAndHoodAngle(3350, 32.5, m_cargo, m_shooter, m_hood, m_robot),
        // new ParallelRaceGroup(
        // new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
        // m_limelight),
        // new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
        // ),
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
            new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
                m_limelight),
            // new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
            new ShootWithSetRPMAndHoodAngle(3000, 30.0, m_cargo, m_shooter, m_hood, m_robot)),
        new InstantCommand(() -> m_hood.setMinAngle()), new InstantCommand(
            () -> m_shooter.setFlywheelRPM(Constants.ShooterConstants.IDLE), m_shooter));
  }

  public Command humanPlayerPracticeAuto(){
    PathPlannerTrajectory path1 = PathPlanner.loadPath("5ball_part3", 4.5, 3.5);
    PathPlannerTrajectory path2 = PathPlanner.loadPath("humanPlayerPracExitTerminal", 4.5, 3.5);

    Pose2d startPose = getStartPoseForPath(path1);
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrain.resetOdometry(startPose)),

        new ParallelCommandGroup(
            // run the intake while we drive forward, but retract after we start driving
            // away from loading zone. vx is robot centric x velocity.
            new IntakeDeployCommand(m_intake, m_cargo)
                .until(() -> (m_drivetrain.getChassisSpeeds().vxMetersPerSecond < -0.5)),
            new SequentialCommandGroup(
                PPSwerveControlCommand(path1, true), 
                new WaitCommand(0.4),
                PPSwerveControlCommand(path2, true)
            )
        ),

        new IntakeReverseCommand(m_intake, m_cargo)
            .withTimeout(2)

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

    Pose2d startPose = getStartPoseForPath(path1);

    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrain.resetOdometry(startPose)),
        new ParallelCommandGroup(
            new IntakeDeployCommand(m_intake, m_cargo)
                .until(() -> (m_cargo.cargoInLowerBelts() && m_cargo.cargoInUpperBelts())),
            PPSwerveControlCommand(path1, true)),
        new InstantCommand(() -> m_shooter.setFlywheelRPM(2800)),
        new InstantCommand(() -> m_hood.setAngleDegrees(29)), PPSwerveControlCommand(path1b, true),
        new ParallelRaceGroup(
            new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
                m_limelight),
            // new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)
            new ShootWithSetRPMAndHoodAngle(2800, 29, m_cargo, m_shooter, m_hood, m_robot)),
        new ParallelCommandGroup(new IntakeDeployCommand(m_intake, m_cargo)
            .until(() -> m_cargo.cargoInLowerBelts()).withTimeout(3.0),
            PPSwerveControlCommand(path2, true)),
        new IntakeReverseCommand(m_intake, m_cargo).withTimeout(2.0),
        PPSwerveControlCommand(path3, true));
  }

//   public Command testShootBall() {
//     return new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot);
//  }


  // -----------------------------------------------CHEZY CHAMPS
  // AUTOS---------------------------------------------------------

  public Command chezyLeft3plus1() {
    // reusing some of the 2plus1ball trajectories as there is an overlap
    PathPlannerTrajectory path1 = PathPlanner.loadPath("2plus1ball_part1", 3.0, 1.5);

    PathPlannerTrajectory path2 = PathPlanner.loadPath("Chezy_3plus1_part2", 3.0, 1.5);

    PathPlannerTrajectory path3 = PathPlanner.loadPath("2plus1ball_part3", 3.0, 1.5);

    Pose2d startPose = getStartPoseForPath(path1);

    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrain.resetOdometry(startPose)),

        // shoot first two preloads
        new ParallelRaceGroup(
            // new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),
            new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
                m_limelight),
            // eventually switch to using a raw value
            // new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
            // m_limelight),
            new ShootWithSetRPMAndHoodAngle(2900, 22, m_cargo, m_shooter, m_hood, m_robot)),

        new InstantCommand(() -> m_shooter.setFlywheelRPM(3300), m_shooter),
        new InstantCommand(() -> m_hood.setAngleDegrees(30), m_hood),

        // drive back and pick up 3rd ball
        new ParallelCommandGroup(new IntakeDeployCommand(m_intake, m_cargo)
            .until(() -> m_cargo.cargoInLowerBelts()).withTimeout(3),
            PPSwerveControlCommand(path1, true)),

        // shoot third ball
        new ParallelRaceGroup(
            // new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),
            new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
                m_limelight),
            // eventually switch to using a raw value
            // new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
            // m_limelight),
            new ShootWithSetRPMAndHoodAngle(3300, 30, m_cargo, m_shooter, m_hood, m_robot)),

        new InstantCommand(() -> m_shooter.setFlywheelRPM(0), m_shooter),
        new InstantCommand(() -> m_hood.setAngleDegrees(0), m_hood),

        // drive with intake down to pick up opponent cargo
        new ParallelCommandGroup(PPSwerveControlCommand(path2, true),
            new IntakeDeployCommand(m_intake, m_cargo).until(() -> m_cargo.cargoInLowerBelts())
                .withTimeout(3.0)),

        // spit out opponent cargo
        new IntakeReverseCommand(m_intake, m_cargo).withTimeout(2.0),

        // start driving to ideal teleop starting point
        PPSwerveControlCommand(path3, true));
  }

  public Command chezyLeft3Plus2() {
    PathPlannerTrajectory path1 = PathPlanner.loadPath("2plus2ball_part1", 3.0, 1.5);

    PathPlannerTrajectory path2 = PathPlanner.loadPath("2plus2ball_part2", 3.0, 1.5);

    PathPlannerTrajectory path3 = PathPlanner.loadPath("2plus2ball_part3", 3.0, 1.5);

    PathPlannerTrajectory path4 = PathPlanner.loadPath("2plus2ball_part4", 3.0, 1.5);

    Pose2d startPose = getStartPoseForPath(path1);

    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrain.resetOdometry(startPose)),

        // shoot first two preloads
        new ParallelRaceGroup(
            // new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),
            new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
                m_limelight),
            // eventually switch to using a raw value
            // new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
            // m_limelight),
            new ShootWithSetRPMAndHoodAngle(2900, 22, m_cargo, m_shooter, m_hood, m_robot)),

        new InstantCommand(() -> m_shooter.setFlywheelRPM(3300), m_shooter),
        new InstantCommand(() -> m_hood.setAngleDegrees(30), m_hood),

        // drive back and pick up 3rd ball
        new ParallelCommandGroup(new IntakeDeployCommand(m_intake, m_cargo)
            .until(() -> m_cargo.cargoInLowerBelts()).withTimeout(3),
            PPSwerveControlCommand(path1, true)),

        // shoot third ball
        new ParallelRaceGroup(
            // new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),
            new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
                m_limelight),
            // eventually switch to using a raw value
            // new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
            // m_limelight),
            new ShootWithSetRPMAndHoodAngle(3300, 30, m_cargo, m_shooter, m_hood, m_robot)),

        new InstantCommand(() -> m_shooter.setFlywheelRPM(0), m_shooter),
        new InstantCommand(() -> m_hood.setAngleDegrees(0), m_hood),


        // drive with intake down to pick up opponent cargo
        new ParallelCommandGroup(PPSwerveControlCommand(path2, true),
            new IntakeDeployCommand(m_intake, m_cargo)
                .until(() -> m_cargo.cargoInLowerBelts() && m_cargo.cargoInUpperBelts())
                .withTimeout(4)),

        // drive to hanger
        PPSwerveControlCommand(path3, true),

        // spit out both opponent cargo
        new IntakeReverseCommand(m_intake, m_cargo).withTimeout(3),

        // drive to ideal teleop start point
        PPSwerveControlCommand(path4, true));
  }

  public Command chezyCenter2ballComplementary(){
    PathPlannerTrajectory path = PathPlanner.loadPath("Chezy_2ball_complamentary", 3.0, 1.5);

    Pose2d startPose = getStartPoseForPath(path);

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrain.resetOdometry(startPose)),
      
      new ParallelCommandGroup(
      new WaitCommand(10.0),

        new SequentialCommandGroup(
      //shoot first 2 preloads 
  
          new ShootWithSetRPMAndHoodAngle(2900, 22, m_cargo, m_shooter, m_hood, m_robot),


          new InstantCommand(() -> m_shooter.setFlywheelRPM(0), m_shooter),
          new InstantCommand(() -> m_hood.setAngleDegrees(0), m_hood)

  
        )
      ),
      //exit tarmac for auto points 
      PPSwerveControlCommand(path, true)
    );
  }

  public Command chezyCenter2ballComplementaryDriveInAndOut(){
    PathPlannerTrajectory path = PathPlanner.loadPath("Chezy_2ball_complamentary", 3.0, 1.5);
    PathPlannerTrajectory path1 = PathPlanner.loadPath("Chezy_2ball_complamentary_reversed", 3.0, 1.5);
    
    Pose2d startPose = getStartPoseForPath(path1);

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrain.resetOdometry(startPose)),
      
      new ShootWithSetRPMAndHoodAngle(2900, 22, m_cargo, m_shooter, m_hood, m_robot),

     new InstantCommand(() -> m_shooter.setFlywheelRPM(0), m_shooter),
     new InstantCommand(() -> m_hood.setAngleDegrees(0), m_hood),

     PPSwerveControlCommand(path, true),

     PPSwerveControlCommand(path1, true)

    );
      
  }
  public Command chezyCenter4ballComplementary() {
    PathPlannerTrajectory path1 = PathPlanner.loadPath("ChezyCenter4ball_part1", 3.0, 1.5);

    PathPlannerTrajectory path2 = PathPlanner.loadPath("ChezyCenter4ball_part2", 3.0, 1.5);

    Pose2d startPose = getStartPoseForPath(path1);

    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrain.resetOdometry(startPose)),

        // shoot first two preloads
        new ParallelRaceGroup(
        //new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),
        // eventually switch to using a raw value
        new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
        m_limelight),
        // m_limelight),
        new ShootWithSetRPMAndHoodAngle(2900, 22, m_cargo, m_shooter, m_hood,
        m_robot)
        ),

        // go to human player terminal to get 2 cargo
        new ParallelCommandGroup(
            // run intake until we go up (y-axis) at 0.5 meters a second
            new IntakeDeployCommand(m_intake, m_cargo)
                .until(() -> m_drivetrain.getChassisSpeeds().vyMetersPerSecond <= -1),

            // go to terminal, wait, drive to shooting position
            new SequentialCommandGroup(PPSwerveControlCommand(path1, true), 
            new WaitCommand(0.8),

            PPSwerveControlCommand(path2, true))),

        // shoot last 2 cargo
        new ParallelRaceGroup(
            //new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),
            new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
                m_limelight),
        // eventually switch to using a raw valu
            new ShootWithSetRPMAndHoodAngle(3100, 31, m_cargo, m_shooter, m_hood,
            m_robot)
        ),

        
     new InstantCommand(() -> m_shooter.setFlywheelRPM(0), m_shooter),
     new InstantCommand(() -> m_hood.setAngleDegrees(0), m_hood)

    );
  }

  public Command chezyRightSide4Ball() {
    PathPlannerTrajectory path1 = PathPlanner.loadPath("5ball_part1", 3.0, 1.5);

    PathPlannerTrajectory path2 = PathPlanner.loadPath("5ball_part2", 3.0, 1.5);

    Pose2d startPose = getStartPoseForPath(path1);

    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrain.resetOdometry(startPose)),

        // shoot first two preloads
        new ParallelRaceGroup(
            new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),
        // eventually switch to using a raw value
         new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
         m_limelight)
        // new ShootWithSetRPMAndHoodAngle(flyWheelRPM, hoodAngle, m_cargo, m_shooter, m_hood,
        // m_robot)
        ),

        new ParallelCommandGroup(
            // intake down to pick up balls
            new IntakeDeployCommand(m_intake, m_cargo)
                .until(() -> m_cargo.cargoInLowerBelts() && m_cargo.cargoInUpperBelts())
                .withTimeout(7),

            new SequentialCommandGroup(
                // pick up first ball
                PPSwerveControlCommand(path1, false),

                // pick up second ball
                PPSwerveControlCommand(path2, true))),

        new ParallelRaceGroup(
            new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot),
        // eventually switch to using a raw value
        new DriveFieldCentricAimCommand(m_drivetrain, () -> 0.0, () -> 0.0, () -> 0.0,
        m_limelight)
        // new ShootWithSetRPMAndHoodAngle(flyWheelRPM, hoodAngle, m_cargo, m_shooter, m_hood,
        // m_robot)
        )
      );

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

    Command swerveControllerCommand =
        new SwerveControllerCommand(trajectory, m_drivetrain::getPose, m_drivetrain.kinematics(),
            new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
            new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
            thetaController, m_drivetrain::setModuleStates, m_drivetrain);

    if (stopAtEnd) {
      // Stop at the end. A good safe default, but not desireable if running two paths back to back
      swerveControllerCommand =
          swerveControllerCommand.andThen(() -> m_drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
    }


    return swerveControllerCommand;
  }

  /**
   * Create a swerve trajectory follow command. If stopAtEnd is set to true, robot will come to full
   * stop when done.
   * 
   * @param trajectory
   * @param stopAtEnd
   * @return
   */
  public static Command PPSwerveControlCommand(PathPlannerTrajectory trajectory,
      boolean stopAtEnd) {

    // var thetaController =
    //     new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController,
    //         AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Command swerveControllerCommand =
    //     new PPSwerveControllerCommand(trajectory, m_drivetrain::getPose, m_drivetrain.kinematics(),
    //         new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
    //         new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
    //         thetaController, m_drivetrain::setModuleStates, m_drivetrain);

    //2023 path planner doesnt take profiled PIDController for thetaController 
    var thetaController =
        new PIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController);

    Command swerveControllerCommand =
        new PPSwerveControllerCommand(
            trajectory, 
            m_drivetrain::getPose, 
            m_drivetrain.kinematics(),
            new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
            new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
            thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

    if (stopAtEnd) {
      // Stop at the end. A good safe default, but not desireable if running two paths back to back
      swerveControllerCommand =
          swerveControllerCommand.andThen(() -> m_drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
    }
    return swerveControllerCommand;
  }
/**
 * This returns the pose2d to reset the odometry to at the start of auto. If you 
 * just use path.getInitalPose() the rotation is the angle of the heading not what the robot is 
 * facing. For swerve robots the rotation value at a state is the 
 * path.getInitialState().holonomicRotation
 * @param path to get the starting pose of 
 * @return the pose2d to reset the odometry to
 */
  public Pose2d getStartPoseForPath(PathPlannerTrajectory path){
      return new Pose2d(path.getInitialPose().getTranslation(), path.getInitialState().holonomicRotation);
  }

}
