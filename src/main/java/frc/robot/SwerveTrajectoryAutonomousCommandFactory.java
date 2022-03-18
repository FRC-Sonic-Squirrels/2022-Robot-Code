// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
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
import frc.robot.commands.ShootCargoCommand;
import frc.robot.commands.ShootOneCargoCommand;
import frc.robot.commands.ShootWithSetRPMCommand;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
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
  private static Robot m_robot;
  private static TestTrajectories m_tt;

  private static int m_shootRPM = 2800;

  public SwerveTrajectoryAutonomousCommandFactory(Drivetrain drivetrain, ShooterSubsystem shooter,
      CargoSubsystem cargo, IntakeSubsystem intake, Robot robot, double maxVelocity, double maxAcceleration) {

    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_cargo = cargo;
    m_intake = intake;
    m_robot = robot;
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
      new ShootWithSetRPMCommand(2750, m_cargo, m_shooter, m_robot)
         .withTimeout(4),
      new ParallelRaceGroup(
        SwerveControllerCommand(moveToCargoOne, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),
        new InstantCommand(() -> m_shooter.setFlywheelRPM(2750), m_shooter),
        SwerveControllerCommand(moveToHub, true),
        new ShootWithSetRPMCommand(2750, m_cargo, m_shooter, m_robot)
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
        new ShootWithSetRPMCommand(2750, m_cargo, m_shooter, m_robot)
          .withTimeout(6)
    );
  }  


 /**
   * twoBallAutoShoot2Push -  drive, pickup cargo, drive back, shoot two, then push opponent cargo out of the way
   */
  public Command twoBallAutoShoot2push(Pose2d startPos, Translation2d cargoPos) {

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    Pose2d targetPose = new Pose2d( cargoPos, startPos.getRotation());
    Pose2d opponentCargoPose = new Pose2d(Constants.FieldConstants.RED_CARGO_4, new Rotation2d(Math.PI/2));
    Pose2d backPose = new Pose2d(Constants.FieldConstants.BEHIND_RED_CARGO_4, new Rotation2d(Math.PI));

    Trajectory moveToCargoOne =  TrajectoryGenerator.generateTrajectory(startPos,
        List.of(), targetPose, m_tt.getTrajectoryConfig());

    Trajectory moveToHub =  TrajectoryGenerator.generateTrajectory(targetPose,
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
        
        // set RPM drive back
        new InstantCommand(() -> m_shooter.setFlywheelRPM(2750), m_shooter),
        SwerveControllerCommand(moveToHub, true),

        // shoot
        new ShootWithSetRPMCommand(2850, m_cargo, m_shooter, m_robot).withTimeout(6),

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
  public Command twoBallAutoWaitShoot2(Double waitTime) {

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
      new WaitCommand(waitTime),
      new InstantCommand(() -> m_shooter.setFlywheelRPM(2750), m_shooter),
      SwerveControllerCommand(moveToHub, true),
      new ShootWithSetRPMCommand(2750, m_cargo, m_shooter, m_robot)
        .withTimeout(6)
    );
  }  

  // command that lets the robot intake 1 cargo without shooting it (parameters have already been converted to meters)
  public Command intakeCargoCommand(TestTrajectories testTrajectories, Translation2d targetPos, Pose2d midPos) {
    
    // precondition: the robot must have at least one free belt
    if (m_cargo.cargoInLowerBelts() && m_cargo.cargoInUpperBelts()) {
      return null;
    }

    Trajectory startToMid = testTrajectories.driveToPose(m_drivetrain.getPose(), midPos);
    //startToMid.transformBy(new Transform2d(drivetrain.getPose(), midPos));
    Trajectory midToTarget = testTrajectories.driveToPose(midPos, new Pose2d(targetPos, midPos.getRotation()));
    //midToTarget.transformBy(new Transform2d(midPos, targetPos));

    return new SequentialCommandGroup(
        // 1. deploy intake then move to the front of a cargo. if there is a stored cargo move it to
        // the upper belts
        new ParallelCommandGroup(
          new CargoMoveToUpperBeltsCommand(m_cargo),
          SwerveControllerCommand(startToMid, true),
          new IntakeDeployCommand(m_intake, m_cargo),
          new WaitUntilCommand(() -> m_intake.intakeAtDesiredRPM()),
          new WaitUntilCommand(() -> !m_cargo.cargoInLowerBelts())),

        // 2. collect the cargo, then wait until it is fully in the lower belts
        SwerveControllerCommand(midToTarget, true),
        new WaitUntilCommand(() -> m_cargo.cargoInLowerBelts()),

        // 3. retract and deactivate the intake
        new ParallelCommandGroup(
          new InstantCommand(() -> m_intake.retractIntake()),
          new InstantCommand(() -> m_intake.stop())));

  }


  /**
   * command for autonomously shooting two cargo then collecting the next two cargo by coordinates.
   * @param startPos the robots position at the beginning of the match (a constant)
   * @param initCargoPos the position of the pre-shoot cargo (meters)
   * @param cargoPos1 the position of the wanted cargo (in meters)
   * @param cargoPos2 the position of the second wanted cargo (meters)
   * @return a set of actions with the robot shooting its current 2 cargo, then moving and picking up the next 2 cargo
   */
  public Command fourBallAutonCommand(Pose2d startPos, Translation2d initCargoPos,
      Translation2d cargoPos1, Translation2d cargoPos2) {

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    // give shootAngle its rotation, then change units
    //Rotation2d shootAngle = new Rotation2d( Math.atan2(shootPos.getY() - HubCentricConstants.HUB_CENTER.y,
    //    shootPos.getX() - HubCentricConstants.HUB_CENTER.x));
    //shootPos = inchesToMeters(shootPos);
    //shootPos = setRotation(shootPos, shootAngle);

    // find the mid poses
    Translation2d initMidPos = midPosFinder(poseToTranslation(startPos), initCargoPos);
    Rotation2d initAngle = getTranslationsAngle(initMidPos, initCargoPos);
    //Pose2d rot_initMidPos = new Pose2d(initMidPos, initAngle);

    Translation2d midPos1 = midPosFinder(poseToTranslation(startPos), cargoPos1);
    Rotation2d angle1 = getTranslationsAngle(midPos1, cargoPos1);
    //Pose2d rot_midPos1 = new Pose2d(midPos1, angle1);
    
    Translation2d midPos2 = midPosFinder(cargoPos1, cargoPos2);
    Rotation2d angle2 = getTranslationsAngle(midPos2, cargoPos2);
    //Pose2d rot_midPos2 = new Pose2d(midPos2, angle2);

    // instantiate all the trajectories
    // Trajectory start_to_initMidPos = m_tt.driveToPose(startPos, rot_initMidPos); // changes angle
    // Trajectory initMidPos_to_initCargoPos = m_tt.driveToPose(initMidPos, initCargoPos); // no angle change
    // Trajectory initCargoPos_to_shootPos = m_tt.driveToPose(initCargoPos, shootPos); // changes angle
    // Trajectory shootPos_to_midPos1 = m_tt.driveToPose(shootPos, rot_midPos1); // changes angle   (pos - pos)
    // Trajectory midPos1_to_cargoPos1 = m_tt.driveToPose(midPos1, cargoPos1); // no angle change   (trans - trans)
    // Trajectory cargoPos1_to_midPos2 = m_tt.driveToPose(cargoPos1, rot_midPos2); // changes angle (trans - pos)
    // Trajectory midPos2_to_cargoPos2 = m_tt.driveToPose(midPos2, cargoPos2); // no angle change   (trans - trans)
    // Trajectory cargoPos2_to_shootPos2 = m_tt.driveToPose(cargoPos2, shootPos2); // changes angle (trans - pos)
    
    Trajectory start_to_initCargo = TrajectoryGenerator.generateTrajectory(
        startPos, List.of(initMidPos), new Pose2d(initCargoPos, initAngle), m_tt.getTrajectoryConfig());

    Trajectory initCargo_to_start = TrajectoryGenerator.generateTrajectory(
        new Pose2d(initCargoPos, initAngle), List.of(), startPos, m_tt.getTrajectoryConfig());

    Trajectory start_to_cargo1 = TrajectoryGenerator.generateTrajectory(
        startPos, List.of(midPos1), new Pose2d(cargoPos1, angle1), m_tt.getTrajectoryConfig());

    Trajectory cargo1_to_cargo2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(cargoPos1, angle1), List.of(midPos2), new Pose2d(cargoPos2, angle2), m_tt.getTrajectoryConfig());

    Trajectory cargo2_to_start = TrajectoryGenerator.generateTrajectory(
        new Pose2d(cargoPos2, angle2), List.of(), startPos, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(
      // 1. deploy intake then wait
      

      // 2. collect initial cargo
      new ParallelCommandGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        SwerveControllerCommand(start_to_initCargo, true)
        // 3. set up flywheel then move to the shooting location
        // new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_activated)),
        // new WaitUntilCommand(() -> m_shooter.isAtDesiredRPM()) ),
      ),
      
      SwerveControllerCommand(initCargo_to_start, false),
      
      // 4. shoot both cargo
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
          .withTimeout(4),
      
      // 5. pick up cargo 1, then wait until it is in the upper belts
      SwerveControllerCommand(start_to_cargo1, true),
      
      // 6. move from where the first cargo was, to the second cargo and collect it
      SwerveControllerCommand(cargo1_to_cargo2, true),

      // 7. wait until it is fully in the lower belts
      //new WaitUntilCommand(() -> m_cargo.cargoInLowerBelts()),

      // 8. after both cargo are loaded, go to the second shoot position while stopping the intake
      new ParallelCommandGroup(
        new InstantCommand(() -> m_intake.retractIntake()),
        new InstantCommand(() -> m_intake.stop()),
        SwerveControllerCommand(cargo2_to_start, true)
      ),

      // 9. shoot the last two cargo
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
          .withTimeout(4)
      // congratulations you (the robot) did it
    );
  }

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

      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
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

      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
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
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
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
        
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
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
      
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
        .withTimeout(4),

      new ParallelRaceGroup(
        SwerveControllerCommand(shoot_to_cargo2, true),
        new IntakeDeployCommand(m_intake, m_cargo)
      ),

      SwerveControllerCommand(cargo2_to_shoot, true),

      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
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
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
        .withTimeout(4),

      SwerveControllerCommand(start_to_cargo, true)
    );
  }


  /**
   * Command for shooting the first cargo, then collecting and shooting a second cargo
   * 
   * @param startPos the initial starting position
   * @param cargoPos the position of the cargo to be picked up
   * @return the trajectory and commands needed to shoot two cargo into the hub
   */
  public Command twoBallAutonCommand(Pose2d startPos, Translation2d cargoPos) {

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    Translation2d midPos = midPosFinder( poseToTranslation(startPos), cargoPos );
    Rotation2d midAngle = getTranslationsAngle( poseToTranslation(startPos), cargoPos );

    Trajectory start_to_cargo = TrajectoryGenerator.generateTrajectory(startPos, List.of(midPos),
        new Pose2d(cargoPos, midAngle), m_tt.getTrajectoryConfig());

    Trajectory cargo_to_start = TrajectoryGenerator.generateTrajectory(new Pose2d(cargoPos, midAngle), List.of(),
        startPos, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(
      // 1. start up intake and flywheel
      new ParallelCommandGroup(
        new IntakeDeployCommand(m_intake, m_cargo)
        // new InstantCommand( () -> m_shooter.setFlywheelRPM(ShooterConstants.m_activated) ),
        // new WaitUntilCommand( () -> m_shooter.isAtDesiredRPM() )
      ),

      // 2. shoot the pre-loaded cargo
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
          .withTimeout(4),

      // 3. move to the next cargo and move it to the upper belts
      new ParallelCommandGroup(
        SwerveControllerCommand(start_to_cargo, true),
        new WaitUntilCommand(() -> m_cargo.cargoInUpperBelts())
      ),

      // 4. after cargo is picked up, move back to the start and retract intake
      new ParallelCommandGroup(
        SwerveControllerCommand(cargo_to_start, true),
        new InstantCommand(() -> m_intake.retractIntake())
      ),

      // 5. shoot the newly picked up cargo
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
          .withTimeout(4)

      // 6. stop flywheel
      //new InstantCommand( () -> m_shooter.setFlywheelRPM(ShooterConstants.m_idle))

    );
  }


  /**
   * Specific command for shooting held cargo and cargo 7, then pushing opposing cargo 4 into hangar.
   * When on blue team, the start pose is BLUE_TOP.
   * When on red team, the start pose is RED_BOTTOM
   * @param team for which alliance the robot is on ("red" or "blue", ignoring case)
   * @return
   */
  public Command complimentaryAutonCommand(String team) {

    Pose2d startPos = null;
    Translation2d cargo7 = null, oppCargo4 = null, pushPos1 = null, pushPos2 = null;
    Rotation2d startAngle = null, shootAngle = null;
    
    // choose coordinates to drive to based off alliance color
    //if (team.equalsIgnoreCase("blue")) {
      //TODO: adjust
      startPos = StartPoseConstants.BLUE_DEF_TOP;
      cargo7 = FieldConstants.BLUE_CARGO_7;
      oppCargo4 = FieldConstants.RED_CARGO_4;
      pushPos1 = new Translation2d(21.5, 23.5);
      pushPos2 = new Translation2d(15, 24.5);
    // }
    // else if (team.equalsIgnoreCase("red")) {
    //   startPos = StartPoseConstants.RED_DEF_BOTTOM;
    //   cargo7 = FieldConstants.RED_CARGO_7;
    //   oppCargo4 = FieldConstants.BLUE_CARGO_4;
    //   pushPos1 = new Translation2d(32.5, 3.5);
    //   pushPos2 = new Translation2d(39, 2.5);
    // }
    // else {
    //   // is this the correct argument exception?
    //   throw new IllegalArgumentException("String parameter was neither \"red\" nor \"blue\".");
    // }

    m_drivetrain.setPose(startPos, m_drivetrain.getIMURotation());

    startAngle = getTranslationsAngle(poseToTranslation(startPos), cargo7);
    startPos = setRotation(startPos, startAngle);

    shootAngle = startPos.getRotation();
    Pose2d shootPos = new Pose2d(poseToTranslation(startPos), shootAngle);


    Trajectory start_to_cargo7 = TrajectoryGenerator.generateTrajectory(startPos, List.of(),
        new Pose2d(cargo7, startPos.getRotation()), m_tt.getTrajectoryConfig());

    Trajectory cargo7_to_shootPos = TrajectoryGenerator.generateTrajectory(new Pose2d(cargo7, startPos.getRotation()),
        List.of(), shootPos, m_tt.getTrajectoryConfig());

    Trajectory shootPos_to_pushPos1 = TrajectoryGenerator.generateTrajectory(shootPos, List.of(),
        new Pose2d(pushPos1, shootAngle), m_tt.getTrajectoryConfig());

    Trajectory pushPos1_to_pushPos2 = TrajectoryGenerator.generateTrajectory(new Pose2d(pushPos1, shootAngle),
        List.of(), new Pose2d(pushPos2, shootAngle), m_tt.getTrajectoryConfig());
    
    return new SequentialCommandGroup(

      // 1. deploy intake and flywheel, while moving to first cargo
      new ParallelCommandGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        //new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_activated)),
        SwerveControllerCommand(start_to_cargo7, true)
      ),

      // 2. move back to start then shoot both contained cargo, while retracting intake
      new ParallelCommandGroup(
        new InstantCommand(() -> m_intake.retractIntake()),
        SwerveControllerCommand(cargo7_to_shootPos, true)
      ),
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
          .withTimeout(4),

      // 3. move to cargo to push, while idling flywheel
      new ParallelCommandGroup(
        //new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_idle)),
        SwerveControllerCommand(shootPos_to_pushPos1, true)
      ),

      // 4. push cargo
      SwerveControllerCommand(pushPos1_to_pushPos2, true)

    );

  }


  /**
   * Create a pre-made 5-ball command set
   * @param alliance "red" or "blue", depending on your team (not case-sensitive)
   * @return
   */
  public Command fiveBallAutonCommand(String alliance) {

    Pose2d startPos_and_shootPos = null, shootPos2 = null, playerMidPos = null;
    Translation2d cargoPos1 = null, cargoPos2 = null, cargoPos3 = null;

    //if (alliance.equalsIgnoreCase("blue")) {

      startPos_and_shootPos = StartPoseConstants.BLUE_DEF_BOTTOM;
      cargoPos1 = FieldConstants.BLUE_CARGO_3;
      cargoPos2 = FieldConstants.BLUE_CARGO_2;
      shootPos2 = StartPoseConstants.BLUE_DEF_BOTTOM;
      cargoPos3 = FieldConstants.BLUE_CARGO_1;
      playerMidPos = new Pose2d(cargoPos3.getX() + 1, cargoPos3.getY() + 1, new Rotation2d(3*Math.PI/4));
    // }
    // else if (alliance.equalsIgnoreCase("red")) {

    //   startPos_and_shootPos = StartPoseConstants.RED_DEF_TOP;
    //   cargoPos1 = FieldConstants.RED_CARGO_3;
    //   cargoPos2 = FieldConstants.RED_CARGO_2;
    //   shootPos2 = StartPoseConstants.RED_DEF_TOP;
    //   cargoPos3 = FieldConstants.RED_CARGO_1;
    //   playerMidPos = new Pose2d(cargoPos3.getX() - 1, cargoPos3.getY() - 1, new Rotation2d(7*Math.PI/4));
    // } else {
    //   throw new IllegalArgumentException("argument was neither \"red\" nor \"blue\"");
    // }

    m_drivetrain.setPose(startPos_and_shootPos, m_drivetrain.getIMURotation());

    Trajectory start_to_cargo1 = TrajectoryGenerator.generateTrajectory(startPos_and_shootPos, List.of(),
        new Pose2d(cargoPos1, getTranslationsAngle(poseToTranslation(startPos_and_shootPos), cargoPos1)), m_tt.getTrajectoryConfig());

    Trajectory cargo1_to_shoot = TrajectoryGenerator.generateTrajectory(new Pose2d(cargoPos1, getTranslationsAngle(poseToTranslation(startPos_and_shootPos), cargoPos1)),
        List.of(), startPos_and_shootPos, m_tt.getTrajectoryConfig());

    Trajectory shoot_to_cargo2 = TrajectoryGenerator.generateTrajectory(startPos_and_shootPos, List.of(),
        new Pose2d(cargoPos2, getTranslationsAngle(poseToTranslation(startPos_and_shootPos), cargoPos2)), m_tt.getTrajectoryConfig());

    Trajectory cargo2_to_shoot2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(cargoPos2, getTranslationsAngle(poseToTranslation(startPos_and_shootPos), cargoPos2)),
        List.of(), shootPos2, m_tt.getTrajectoryConfig());
    
    Trajectory shoot2_to_playerMid = TrajectoryGenerator.generateTrajectory(shootPos2, List.of(),
        playerMidPos, m_tt.getTrajectoryConfig());
    
    Trajectory playerMid_to_cargo3 = TrajectoryGenerator.generateTrajectory(playerMidPos, List.of(),
        new Pose2d(cargoPos3, playerMidPos.getRotation()), m_tt.getTrajectoryConfig());
    
    Trajectory cargo3_to_shoot2 = TrajectoryGenerator.generateTrajectory(new Pose2d(cargoPos3, playerMidPos.getRotation()),
        List.of(), shootPos2, m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(

      // 1. start up intake + flywheel, move to first cargo
      new ParallelCommandGroup(
        new IntakeDeployCommand(m_intake, m_cargo),
        //new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_activated)),
        SwerveControllerCommand(start_to_cargo1, true)
      ),

      // 2. move back to start then shoot both cargo
      SwerveControllerCommand(cargo1_to_shoot, true),
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
          .withTimeout(4),

      // 3. move to and pick up next cargo, then go back to shoot that one cargo
      SwerveControllerCommand(shoot_to_cargo2, true),
      SwerveControllerCommand(cargo2_to_shoot2, true),
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
          .withTimeout(4),

      // 4. go to human player area to pick up two new cargo
      SwerveControllerCommand(shoot2_to_playerMid, true),
      SwerveControllerCommand(playerMid_to_cargo3, true),

      // 5. move back to shooting area then shoot, while retracting intake
      new ParallelCommandGroup(
        SwerveControllerCommand(cargo3_to_shoot2, true),
        new InstantCommand(() -> m_intake.retractIntake())
      ),
      new ShootWithSetRPMCommand(m_shootRPM, m_cargo, m_shooter, m_robot)
          .withTimeout(4)

      // 6. idle flywheel
      //new InstantCommand(() -> m_shooter.setFlywheelRPM(ShooterConstants.m_idle))
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

    Rotation2d cargoAngle = new Rotation2d(getTranslationsAngleDouble(ejectPos, oppCargo));

    Trajectory moveToEjectPos = TrajectoryGenerator.generateTrajectory(startPos, List.of(),
        new Pose2d(ejectPos, cargoAngle), m_tt.getTrajectoryConfig());

    return new SequentialCommandGroup(

      // 1. deploy intake
      new IntakeDeployCommand(m_intake, m_cargo),

      // 2. move to eject pos
      SwerveControllerCommand(moveToEjectPos, true),

      // 3. release held cargo in the direction of the other cargo (through the intake) and wait for it being fully released
      new ParallelRaceGroup(
        new IntakeReverseCommand(m_intake, m_cargo),
        new SequentialCommandGroup(
          new WaitUntilCommand( () -> !m_cargo.cargoInLowerBelts() ),
          new WaitCommand(2)
        )
      ),

      // 4. retract intake
      new InstantCommand(() -> m_intake.retractIntake())
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
