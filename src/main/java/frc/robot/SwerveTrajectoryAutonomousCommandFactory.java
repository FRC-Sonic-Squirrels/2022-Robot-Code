// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StartPoseConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.CargoMoveToUpperBeltsCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.LimelightRotateToHubAndShoot;
import frc.robot.commands.ShootOneCargoCommand;
import frc.robot.commands.ShootWithSetRPMandSetHoodCommand;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SwerveTrajectoryAutonomousCommandFactory {

  private static ShooterSubsystem m_shooter;
  private static Drivetrain m_drivetrain;
  private static CargoSubsystem m_cargo;
  private static IntakeSubsystem m_intake;
  private static HoodSubsystem m_hood;
  private static Robot m_robot;
  private static TestTrajectories m_tt;
  private static LimelightSubsystem m_limelight;

  private static int m_shootRPM = 3000;


  public SwerveTrajectoryAutonomousCommandFactory(Drivetrain drivetrain, ShooterSubsystem shooter,
      CargoSubsystem cargo, IntakeSubsystem intake, HoodSubsystem hood, Robot robot, LimelightSubsystem limelight, double maxVelocity, double maxAcceleration) {

    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_cargo = cargo;
    m_intake = intake;
    m_hood = hood;
    m_robot = robot;
    m_limelight = limelight;
    m_tt = new TestTrajectories(maxVelocity, maxAcceleration, m_drivetrain, true);
  }

  /**
   * twoBallAuton - Shoot, drive, pickup cargo, drive back, shoot
   */
  public Command twoBallAuto(Pose2d startPos, Translation2d cargoPos) {

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    Pose2d targetPose = new Pose2d( cargoPos, startPos.getRotation());

    Trajectory moveToCargoOne =  TrajectoryGenerator.generateTrajectory(startPos,
        List.of(), targetPose, m_tt.getTrajectoryConfig());

    Trajectory moveToHub =  TrajectoryGenerator.generateTrajectory(targetPose,
        List.of(), startPos, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(      
      new ShootWithSetRPMandSetHoodCommand(2750, 15, m_cargo, m_shooter, m_hood)
         .withTimeout(4),
      new ParallelRaceGroup(
        SwerveControllerCommand(moveToCargoOne, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),
        new InstantCommand(() -> m_shooter.setFlywheelRPM(2750), m_shooter),
        SwerveControllerCommand(moveToHub, true),
        new ShootWithSetRPMandSetHoodCommand(2750, 15, m_cargo, m_shooter, m_hood)
          .withTimeout(6)
    );
  }


  /**
   * twoBallAutoShoot2 -  drive, pickup cargo, drive back, shoot two
   */
  public Command twoBallAutoShoot2(Pose2d startPos, Translation2d cargoPos) {

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    Pose2d targetPose = new Pose2d( cargoPos, startPos.getRotation());

    Trajectory moveToCargoOne =  TrajectoryGenerator.generateTrajectory(startPos,
        List.of(), targetPose, m_tt.getTrajectoryConfig());

    Trajectory moveToHub =  TrajectoryGenerator.generateTrajectory(targetPose,
        List.of(), startPos, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(      
      new ParallelRaceGroup(
        SwerveControllerCommand(moveToCargoOne, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),
        new InstantCommand(() -> m_shooter.setFlywheelRPM(2750), m_shooter),
        SwerveControllerCommand(moveToHub, true),
        new ShootWithSetRPMandSetHoodCommand(2750, 15, m_cargo, m_shooter, m_hood)
          .withTimeout(6)
    );
  }  


 /**
   * twoBallAutoShoot2Push -  drive, pickup cargo, drive back, shoot two, then push opponent cargo out of the way
   */
  public Command twoBallAutoShoot2push() {

    Pose2d startPos = StartPoseConstants.BLUE_DEF_TOP;
    Translation2d cargoPos = FieldConstants.BLUE_CARGO_7;

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

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
        new LimelightRotateToHubAndShoot(m_limelight, m_drivetrain, m_cargo, m_shooter, m_hood),

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

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    Trajectory moveToCargoOne =  TrajectoryGenerator.generateTrajectory(startPos,
        List.of(), cargoPos, m_tt.getTrajectoryConfig());

    Trajectory moveToHub =  TrajectoryGenerator.generateTrajectory(cargoPos,
        List.of(), shootPos, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(      
      new ParallelRaceGroup(
        SwerveControllerCommand(moveToCargoOne, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),
      //new WaitCommand(waitTime),
      new InstantCommand(() -> m_shooter.setFlywheelRPM(2750), m_shooter),
      SwerveControllerCommand(moveToHub, true),
      new ShootWithSetRPMandSetHoodCommand(2750, 15, m_cargo, m_shooter, m_hood)
        .withTimeout(6)
    );
  }  

  // // command that lets the robot intake 1 cargo without shooting it (parameters have already been converted to meters)
  // public Command intakeCargoCommand(TestTrajectories testTrajectories, Translation2d targetPos, Pose2d midPos) {
    
  //   // precondition: the robot must have at least one free belt
  //   if (m_cargo.cargoInLowerBelts() && m_cargo.cargoInUpperBelts()) {
  //     return null;
  //   }

  //   Trajectory startToMid = testTrajectories.driveToPose(m_drivetrain.getPose(), midPos);
  //   //startToMid.transformBy(new Transform2d(drivetrain.getPose(), midPos));
  //   Trajectory midToTarget = testTrajectories.driveToPose(midPos, new Pose2d(targetPos, midPos.getRotation()));
  //   //midToTarget.transformBy(new Transform2d(midPos, targetPos));

  //   return new SequentialCommandGroup(
  //       // 1. deploy intake then move to the front of a cargo. if there is a stored cargo move it to
  //       // the upper belts
  //       new ParallelCommandGroup(
  //         new CargoMoveToUpperBeltsCommand(m_cargo),
  //         SwerveControllerCommand(startToMid, true),
  //         new IntakeDeployCommand(m_intake, m_cargo),
  //         new WaitUntilCommand(() -> m_intake.intakeAtDesiredRPM()),
  //         new WaitUntilCommand(() -> !m_cargo.cargoInLowerBelts())),

  //       // 2. collect the cargo, then wait until it is fully in the lower belts
  //       SwerveControllerCommand(midToTarget, true),
  //       new WaitUntilCommand(() -> m_cargo.cargoInLowerBelts()),

  //       // 3. retract and deactivate the intake
  //       new ParallelCommandGroup(
  //         new InstantCommand(() -> m_intake.retractIntake()),
  //         new InstantCommand(() -> m_intake.stop())));

  // }

  public Command SundomefourBallAutonCommand() {

    Pose2d startPos = Constants.StartPoseConstants.BLUE_BOTTOM;
    Pose2d shootPos = Constants.StartPoseConstants.BLUE_DEF_BOTTOM;

    Translation2d cargoPos1 = Constants.FieldConstants.BLUE_CARGO_3;
    Translation2d cargoPos2 = Constants.FieldConstants.BLUE_CARGO_2;
    Translation2d cargoPos3 = Constants.FieldConstants.BLUE_CARGO_1;

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    
    Trajectory start_to_cargo1 = TrajectoryGenerator.generateTrajectory(
        startPos, List.of(), new Pose2d(cargoPos1, startPos.getRotation()), m_tt.getTrajectoryConfig());

    Trajectory cargo1_to_shoot = TrajectoryGenerator.generateTrajectory(
        new Pose2d(cargoPos1, startPos.getRotation()), List.of(), shootPos, m_tt.getTrajectoryConfig());

    Trajectory shoot_to_cargo2 = TrajectoryGenerator.generateTrajectory(
        shootPos, List.of(), new Pose2d(cargoPos2, Rotation2d.fromDegrees(180)), m_tt.getTrajectoryConfig());

    Trajectory cargo2_to_cargo3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(cargoPos2, Rotation2d.fromDegrees(180)), List.of(), 
      new Pose2d(cargoPos3, Rotation2d.fromDegrees(180)), m_tt.getTrajectoryConfig());

    Trajectory cargo3_to_shoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(cargoPos3, Rotation2d.fromDegrees(180)), List.of(), shootPos, m_tt.getTrajectoryConfig());
    

    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        SwerveControllerCommand(start_to_cargo1, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),
      SwerveControllerCommand(cargo1_to_shoot, true),

      new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood)
        .withTimeout(4),

      new ParallelRaceGroup(
        SwerveControllerCommand(shoot_to_cargo2, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),
        
      new ParallelRaceGroup(
        SwerveControllerCommand(cargo2_to_cargo3, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),

      SwerveControllerCommand(cargo3_to_shoot, true),

      new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood)
        .withTimeout(4)
    );
  }

  /**
   * Command for shooting the first cargo, then collecting and shooting two more cargo
   * @param startPos the initial starting position
   * @param cargoPos1 position of the first cargo being picked up
   * @param cargoPos2 position of the second cargo being picked up
   * @return the trajectory and commands needed to shoot three cargo into the hub
   */
  public Command SundomethreeBallAutonShootFirstCommand() {
    Pose2d startPos = Constants.StartPoseConstants.BLUE_DEF_BOTTOM; //bottom fender
    Translation2d cargoPos1 = new Translation2d( Units.inchesToMeters(297.6), Units.inchesToMeters(7.2) + 25);
    Translation2d cargoPos2 = Constants.FieldConstants.BLUE_CARGO_2;


    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    Trajectory start_to_cargo1 = TrajectoryGenerator.generateTrajectory(
      startPos, 
      List.of(),
      new Pose2d(cargoPos1, Rotation2d.fromDegrees(270)), m_tt.getTrajectoryConfig());

    Trajectory cargo1_to_cargo2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(cargoPos1, Rotation2d.fromDegrees(270)), 
      List.of(),
      new Pose2d(cargoPos2, Rotation2d.fromDegrees(135)), m_tt.getTrajectoryConfig());

    Trajectory cargo2_to_start = TrajectoryGenerator.generateTrajectory(
      new Pose2d(cargoPos2, Rotation2d.fromDegrees(135)), 
      List.of(new Translation2d(Units.feetToMeters(4), 0)),
      startPos, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(
      new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood)
          .withTimeout(4),
      
      new ParallelRaceGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        SwerveControllerCommand(start_to_cargo1, true)
      ),
      
      new ParallelCommandGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        SwerveControllerCommand(cargo1_to_cargo2, true)
      ),


     SwerveControllerCommand(cargo2_to_start, true),
        
      new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood)
          .withTimeout(4)
    );
  }

  /**
   * Command for shooting the first cargo, then collecting and shooting two more cargo
   * @param startPos the initial starting position
   * @param cargoPos1 position of the first cargo being picked up
   * @param cargoPos2 position of the second cargo being picked up
   * @return the trajectory and commands needed to shoot three cargo into the hub
   */
  public Command SundomethreeBallAutonMoveFirstCommand() {
    Pose2d startPos = Constants.StartPoseConstants.BLUE_BOTTOM;
    Pose2d shootingPos = Constants.StartPoseConstants.BLUE_DEF_BOTTOM;
    //cargo pos -16 inches y-axis to avoid intake hitting the wall 
    Translation2d cargoPos1 = new Translation2d( Units.inchesToMeters(297.6), Units.inchesToMeters(7.2) + 25); 
    Translation2d cargoPos2 = Constants.FieldConstants.BLUE_CARGO_2;

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    Trajectory start_to_cargo1 = TrajectoryGenerator.generateTrajectory(
      startPos, 
      List.of(),
      new Pose2d(cargoPos1, startPos.getRotation()), m_tt.getTrajectoryConfig());

    Trajectory cargo1_to_shoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(cargoPos1, shootingPos.getRotation()),
      List.of(), 
      startPos, m_tt.getTrajectoryConfig());

    Trajectory shoot_to_cargo2 = TrajectoryGenerator.generateTrajectory(
    shootingPos,
    List.of(), 
    new Pose2d(cargoPos2, Rotation2d.fromDegrees(210)), m_tt.getTrajectoryConfig());

    Trajectory cargo2_to_shoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(cargoPos2, Rotation2d.fromDegrees(210)), 
      List.of(),
      shootingPos, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        SwerveControllerCommand(start_to_cargo1, true)
      ),

      SwerveControllerCommand(cargo1_to_shoot, true),
      
      new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood)
        .withTimeout(4),

      new ParallelRaceGroup(
        SwerveControllerCommand(shoot_to_cargo2, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),

      SwerveControllerCommand(cargo2_to_shoot, true),

      new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood)
          .withTimeout(4)
    );
  }

  public Command SundomeRightSideShootAndMove(){
    Pose2d startPos = Constants.StartPoseConstants.BLUE_DEF_BOTTOM;
    Translation2d cargoPos = Constants.FieldConstants.BLUE_CARGO_2;

    Trajectory start_to_cargo = TrajectoryGenerator.generateTrajectory(
      startPos, List.of(), 
      new Pose2d(cargoPos, startPos.getRotation()), m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(
      new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood)
        .withTimeout(4),

      SwerveControllerCommand(start_to_cargo, true)
    );
  }


  // /**
  //  * Command for shooting the first cargo, then collecting and shooting a second cargo
  //  * 
  //  * @param startPos the initial starting position
  //  * @param cargoPos the position of the cargo to be picked up
  //  * @return the trajectory and commands needed to shoot two cargo into the hub
  //  */
  // public Command twoBallAutonCommand(Pose2d startPos, Translation2d cargoPos) {

  //   m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

  //   Translation2d midPos = midPosFinder( poseToTranslation(startPos), cargoPos );
  //   Rotation2d midAngle = getTranslationsAngle( poseToTranslation(startPos), cargoPos );

  //   Trajectory start_to_cargo = TrajectoryGenerator.generateTrajectory(startPos, List.of(midPos),
  //       new Pose2d(cargoPos, midAngle), m_tt.getTrajectoryConfig());

  //   Trajectory cargo_to_start = TrajectoryGenerator.generateTrajectory(new Pose2d(cargoPos, midAngle), List.of(),
  //       startPos, m_tt.getTrajectoryConfig());

  //   return new SequentialCommandGroup(
  //     // 1. start up intake and flywheel
  //     new ParallelCommandGroup(
  //       new IntakeDeployCommand(m_intake, m_cargo)
  //       // new InstantCommand( () -> m_shooter.setFlywheelRPM(ShooterConstants.m_activated) ),
  //       // new WaitUntilCommand( () -> m_shooter.isAtDesiredRPM() )
  //     ),

  //     // 2. shoot the pre-loaded cargo
  //     new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood, m_robot)
  //         .withTimeout(4),

  //     // 3. move to the next cargo and move it to the upper belts
  //     new ParallelCommandGroup(
  //       SwerveControllerCommand(start_to_cargo, true),
  //       new WaitUntilCommand(() -> m_cargo.cargoInUpperBelts())
  //     ),

  //     // 4. after cargo is picked up, move back to the start and retract intake
  //     new ParallelCommandGroup(
  //       SwerveControllerCommand(cargo_to_start, true),
  //       new InstantCommand(() -> m_intake.retractIntake())
  //     ),

  //     // 5. shoot the newly picked up cargo
  //     new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood, m_robot)
  //         .withTimeout(4)

  //     // 6. stop flywheel
  //     //new InstantCommand( () -> m_shooter.setFlywheelRPM(ShooterConstants.m_idle))

  //   );
  // }

  // /**
  //  * Create a pre-made 5-ball command set
  //  */
  // public Command fiveBallAutonCommand() {

  //   Pose2d startPos_and_shootPos = StartPoseConstants.BLUE_DEF_BOTTOM;

  //   Pose2d cargoPos1 = new Pose2d(FieldConstants.BLUE_CARGO_3,
  //       getTranslationsAngle( poseToTranslation(startPos_and_shootPos), FieldConstants.BLUE_CARGO_3 ));

  //   Pose2d cargoPos2 = new Pose2d(FieldConstants.BLUE_CARGO_2,
  //       getTranslationsAngle( poseToTranslation(startPos_and_shootPos), FieldConstants.BLUE_CARGO_2 ));

  //   Pose2d shootPos2 = StartPoseConstants.BLUE_DEF_BOTTOM;

  //   Pose2d playerMidPos = new Pose2d(FieldConstants.BLUE_CARGO_1.getX() + 1, FieldConstants.BLUE_CARGO_1.getY() + 1,
  //       new Rotation2d(3*Math.PI/4));

  //   Pose2d cargoPos3 = new Pose2d(FieldConstants.BLUE_CARGO_1,
  //       new Rotation2d(3*Math.PI/4));


  //   m_drivetrain.setPose(startPos_and_shootPos, m_drivetrain.getIMURotation());

  //   Trajectory start_to_cargo1 = TrajectoryGenerator.generateTrajectory(startPos_and_shootPos, List.of(),
  //       cargoPos1, m_tt.getTrajectoryConfig());

  //   Trajectory cargo1_to_shoot = TrajectoryGenerator.generateTrajectory(cargoPos1, List.of(),
  //       startPos_and_shootPos, m_tt.getTrajectoryConfig());

  //   Trajectory shoot_to_cargo2 = TrajectoryGenerator.generateTrajectory(startPos_and_shootPos, List.of(),
  //       cargoPos2, m_tt.getTrajectoryConfig());

  //   Trajectory cargo2_to_shoot2 = TrajectoryGenerator.generateTrajectory(cargoPos2, List.of(),
  //       shootPos2, m_tt.getTrajectoryConfig());
    
  //   Trajectory shoot2_to_playerMid = TrajectoryGenerator.generateTrajectory(shootPos2, List.of(),
  //       playerMidPos, m_tt.getTrajectoryConfig());
    
  //   Trajectory playerMid_to_cargo3 = TrajectoryGenerator.generateTrajectory(playerMidPos, List.of(),
  //       cargoPos3, m_tt.getTrajectoryConfig());
    
  //   Trajectory cargo3_to_shoot2 = TrajectoryGenerator.generateTrajectory(cargoPos3,
  //       List.of(), shootPos2, m_tt.getTrajectoryConfig());

  //   return new SequentialCommandGroup(

  //     // 1. start up intake + flywheel, move to first cargo
  //     new ParallelRaceGroup(
  //       new IntakeDeployCommand(m_intake, m_cargo),
  //       //new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_activated)),
  //       SwerveControllerCommand(start_to_cargo1, true)
  //     ),

  //     // 2. move back to start then shoot both cargo
  //     SwerveControllerCommand(cargo1_to_shoot, true),
  //     new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood, m_robot)
  //         .withTimeout(4),

  //     // 3. move to and pick up next cargo, then go back to shoot that one cargo
  //     SwerveControllerCommand(shoot_to_cargo2, true),
  //     SwerveControllerCommand(cargo2_to_shoot2, true),
  //     new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood, m_robot)
  //         .withTimeout(4),

  //     // 4. go to human player area to pick up two new cargo
  //     SwerveControllerCommand(shoot2_to_playerMid, true),
  //     SwerveControllerCommand(playerMid_to_cargo3, true),

  //     // 5. move back to shooting area then shoot, while retracting intake
  //     new ParallelCommandGroup(
  //       SwerveControllerCommand(cargo3_to_shoot2, true),
  //       new InstantCommand(() -> m_intake.retractIntake())
  //     ),
  //     new ShootWithSetRPMandSetHoodCommand(m_shootRPM, 15, m_cargo, m_shooter, m_hood, m_robot)
  //         .withTimeout(4)

  //     // 6. idle flywheel
  //     //new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_idle))

  //   );
  // }

  /**
   * creates the pre-made 5 ball auton command, using trajectories from PathPlanner
   */
  public Command better5BallAuton() {

    // drivetrain.setPose();
    //Trajectory test = PathPlanner.loadPath("5ball_part1", 1.0, 0.75);

    Trajectory traject1 = null;
    Trajectory traject2 = null;
    Trajectory traject3 = null;
    Trajectory traject4 = null;

    return new SequentialCommandGroup(

      new ParallelRaceGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        SwerveControllerCommand(traject1, true)
      ),
      new LimelightRotateToHubAndShoot(m_limelight, m_drivetrain, m_cargo, m_shooter, m_hood),

      new ParallelRaceGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        SwerveControllerCommand(traject2, true)
      ),
      new LimelightRotateToHubAndShoot(m_limelight, m_drivetrain, m_cargo, m_shooter, m_hood),

      SwerveControllerCommand(traject3, true),
      new IntakeDeployCommand(m_intake, m_cargo).withTimeout(4),

      SwerveControllerCommand(traject4, true),
      new LimelightRotateToHubAndShoot(m_limelight, m_drivetrain, m_cargo, m_shooter, m_hood)

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

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    // get the angle between the pose of the robot and the location of the cargo
    Rotation2d cargoAngle = new Rotation2d(getTranslationsAngleDouble(ejectPos, oppCargo));

    Trajectory moveToEjectPos = TrajectoryGenerator.generateTrajectory(startPos, List.of(),
        new Pose2d(ejectPos, cargoAngle), m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(

      SwerveControllerCommand(moveToEjectPos, true),

      new ParallelRaceGroup(
        new IntakeReverseCommand(m_intake, m_cargo),
        new WaitCommand(4)
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
