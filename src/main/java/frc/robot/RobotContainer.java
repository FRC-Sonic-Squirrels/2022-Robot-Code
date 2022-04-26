// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.ArrayList;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2930.lib.OrchestraMusicController;
import com.team2930.lib.OrchestraMusicController.Playlists;
import com.team2930.lib.OrchestraMusicController.Songs;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.StartPoseConstants;
import frc.robot.commands.ArmManualControlCommand;
import frc.robot.commands.CargoReverseCommand;
import frc.robot.commands.CargoRunIndexer;
import frc.robot.commands.ClimbAutoMid;
import frc.robot.commands.ControllerClimbMaxHeightRumble;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.DriveFieldCentricAimCommand;
import frc.robot.commands.DriveFieldCentricCommand;
import frc.robot.commands.DriveFieldCentricHoldAngle;
import frc.robot.commands.DriveWithSetRotationCommand;
import frc.robot.commands.ElevatorControlCommand;
import frc.robot.commands.ElevatorGoToMaxHeight;
import frc.robot.commands.HoodZeroAngle;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.LimelightAutoShoot;
import frc.robot.commands.ShootManualAdjustRpmAndAngle;
import frc.robot.commands.ShootWithSetRPMAndHoodAngle;
import frc.robot.commands.AutoClimbCommands.ClimbElevatorTest;
import frc.robot.commands.AutoClimbCommands.ClimbFullCommand;
import frc.robot.commands.AutoClimbCommands.ClimbHighFull;
import frc.robot.commands.AutoClimbCommands.ClimbHighToTraverse;
import frc.robot.commands.AutoClimbCommands.ClimbMidAuto;
import frc.robot.commands.AutoClimbCommands.ClimbMidToHigh;
import frc.robot.commands.MusicCommands.OrchestraPlayQueueCommand;
import frc.robot.commands.MusicCommands.OrchestraPlaySongCommand;
import frc.robot.commands.DriveHubCentricCommand;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final Robot m_robot;

  // Subsystems
  public final CargoSubsystem m_cargo;
  public final ShooterSubsystem m_shooter;
  public ElevatorSubsystem m_elevator;
  public ArmSubsystem m_arm;
  public final Drivetrain drivetrain;
  public final IntakeSubsystem m_intake;
  public final HoodSubsystem m_hood;
  public LimelightSubsystem m_limelight;

  // Controllers
  public final XboxController m_controller = new XboxController(0);
  public final XboxController m_operatorController = new XboxController(1);
  public final XboxController m_climbController = new XboxController(2);

  public final SendableChooser<Command> chooser = new SendableChooser<>();
  
  public DriverStation.Alliance m_alliance = DriverStation.getAlliance();

  public double m_shootingRpm = Constants.ShooterConstants.BUMPER_SHOT_RPM;
  public double m_hoodAngle = Constants.ShooterConstants.HOOD_ANGLE;

  public Command climbRumbleCommand = new ControllerClimbMaxHeightRumble(m_climbController, m_elevator);

  Subsystem[] instrumentSubsystems = new Subsystem[2];
  ArrayList<WPI_TalonFX> instrumentMotors;

  OrchestraMusicController musicController;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {

    m_robot = robot;

    m_cargo = new CargoSubsystem();
    m_shooter = new ShooterSubsystem(m_robot);
    m_intake = new IntakeSubsystem();
    drivetrain = new Drivetrain();
    m_elevator = new ElevatorSubsystem();
    m_arm = new ArmSubsystem();
    m_hood = new HoodSubsystem();
    m_limelight = new LimelightSubsystem(drivetrain, m_robot.revPDH);

    instrumentSubsystems[0] = m_elevator;
    instrumentSubsystems[1] = m_hood;

    instrumentMotors = new ArrayList<>();

    instrumentMotors.addAll(m_elevator.getAllMotors());
    instrumentMotors.addAll(m_hood.getAllMotors());

    musicController = OrchestraMusicController.OrchestraFromWpiTalonControllers(instrumentMotors);
    
    SmartDashboard.putData("Auto Mode", chooser);

    // add the new auton trajectories to the auton trajectory chooser
    SwerveTrajectoryAutonomousCommandFactory auton =
        new SwerveTrajectoryAutonomousCommandFactory(drivetrain, m_shooter, m_cargo, m_intake, m_hood,
            m_limelight, m_robot, Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);


    chooser.addOption("nothing", new InstantCommand());

    // test commands
    // Command testCurve = auton.curve();
    // Command testStraightLine = auton.straightLine();
    // Command testLimelightShoot = auton.testShootBall();
    // chooser.addOption("test curve", testCurve);
    // chooser.addOption("test straight line", testStraightLine);
    // chooser.addOption("test limelight shoot", testLimelightShoot);


    // Competition Autonomous
    Command rightSide5Ball = auton.rightSideFiveBall();
    Command left2plus1 = auton.leftSide2plus1();
    Command middle_1Ball_Complementary = auton.middleShootFenderAndLeave();

    chooser.addOption("right side 5 ball", rightSide5Ball);
    chooser.addOption("left side 2 plus 1", left2plus1);
    chooser.addOption("Middle 1ball Complementary", middle_1Ball_Complementary);

    chooser.setDefaultOption("right side 5 ball", rightSide5Ball);

    drivetrain.setDefaultCommand(new DriveFieldCentricCommand(
      drivetrain, 
      () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 
      () -> -modifyAxis(m_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    m_elevator.setDefaultCommand(new ElevatorControlCommand(m_elevator, m_climbController,
      Constants.ElevatorConstants.elevatorSpeedMultiplier));
    
    //new ControllerClimbMaxHeightRumble(m_climbController, m_elevator);

    m_arm.setDefaultCommand(new ArmManualControlCommand(m_arm, m_climbController, 0.3));

    CommandScheduler.getInstance().schedule(false, new ControllerClimbMaxHeightRumble(m_climbController, m_elevator));


    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {

    new Button(m_climbController::getXButton)
      .whenPressed(
        new OrchestraPlayQueueCommand(musicController, Playlists.testPlaylist.files, instrumentSubsystems)
      );

    new Button(m_climbController::getYButton)
      .toggleWhenPressed(
        new OrchestraPlaySongCommand(musicController, Songs.testSong.filepath, instrumentSubsystems)
      );
    //-------------- DRIVER CONTROLS DEFINED HERE --------------------------  

    // new Button(m_controller::getRightBumper)
    //   .whenPressed(new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot));

    new Button(m_controller::getLeftBumper)
      .whenPressed(new ParallelRaceGroup(
        new DriveFieldCentricAimCommand(drivetrain, 
            () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            m_limelight),
        new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot).andThen(new ControllerRumbleCommand(m_controller, 0.2))));

    // Back button resets field centric, forward is the current heading
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(drivetrain::resetFieldCentric);

    // start button toggles the LimeLight LEDs
    new Button(m_controller::getStartButton)
            .whenPressed(new InstantCommand(() -> m_limelight.toggleLEDs()));

    // new Button(m_controller::getXButton)
    //         .whenPressed(new DriveHubCentricCommand(drivetrain, 
    //         () -> -modifyAxis(m_controller.getRightX()), 
    //         () -> -modifyAxis(m_controller.getLeftY())));


    //Temporary to help testing ll shooting 
    // new Button(m_controller::getRightBumper)
    //   .toggleWhenPressed(new IntakeDeployCommand(m_intake, m_cargo));

    // new Button(m_controller::getYButton)
    //   .whileHeld(new IntakeReverseCommand(m_intake, m_cargo));
  
    // new Button(m_controller::getXButton).whenPressed(
    //     new ParallelCommandGroup(
    //       new DriveFieldCentricAimCommand(drivetrain,
    //         () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //         m_limelight),
    //       new LimelightAutoShoot(m_limelight, m_cargo, m_shooter, m_hood, m_robot)));

    new Button(m_controller::getYButton)
        .whenPressed(new DriveFieldCentricAimCommand(drivetrain,
            () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            m_limelight));

    // new Button(m_controller::getYButton)
    //         .whenPressed(new DriveWithSetRotationCommand(drivetrain,
    //         () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> m_controller.getPOV(), 0.0));

    // new Button(m_controller::getXButton)
    //         .whenPressed(new DriveFieldCentricHoldAngle(drivetrain,
    //         () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 
    //         () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    new Button(m_controller::getBButton)
            .whenPressed(new DriveRobotCentricCommand(drivetrain,
            () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.8, 
            () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.8,
            () -> -modifyAxis(m_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    new Button(m_controller::getAButton)
            .whenPressed(new DriveFieldCentricCommand(drivetrain,
            () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 
            () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

  // fender sohpt
    new Button(m_controller::getRightBumper)
    .whenPressed(new ShootWithSetRPMAndHoodAngle(2800, 15, m_cargo, m_shooter, m_hood, m_robot), true);

  // launch pad shot
    new Button (() -> m_controller.getRightTriggerAxis() > 0.05)
     .whenPressed(new ShootWithSetRPMAndHoodAngle(4000, 32, m_cargo, m_shooter, m_hood, m_robot), true);

    // new Button(m_controller::getLeftBumper)
    //         .whileHeld(new DriveChimpMode(drivetrain, m_intake,
    //         () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 
    //         () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

              
    // new Button(m_controller::getLeftBumper)
    //         .whileHeld(new LimelightRotateToHubAndShoot(2000, m_limelight, drivetrain, m_cargo, m_shooter, m_intake, m_robot));

            
    // new Button(() -> (m_controller.getRightTriggerAxis() > 0.05))
    //         .toggleWhenActive(new IntakeDeployCommand(m_intake, m_cargo), true);

    // new Button(() -> (m_controller.getLeftTriggerAxis() > 0.05))
    //         .whileHeld(new IntakeReverseCommand(m_intake, m_cargo));

    // new Button(m_controller::getLeftBumper)
    //   .whileHeld(new VisionRotateToCargo(m_visionSubsystem, drivetrain));

    // new Button(m_controller::getRightBumper)
    //   .whileHeld(new VisionDriveToCargo(m_visionSubsystem, drivetrain));

    // **************** OPERATOR CONTROLS ********************************

    //--------------------------------Operator intake)-------------------
    //Deploy Intake
    new Button(m_operatorController::getAButton)
       .toggleWhenPressed(new IntakeDeployCommand(m_intake, m_cargo));

    new Button(m_operatorController::getYButton)
       .whileHeld(new IntakeReverseCommand(m_intake, m_cargo));

    // // fender sohpt
    // new Button(m_operatorController::getRightBumper)
    //    .whenPressed(new ShootWithSetRPMAndHoodAngle(2800, 15, m_cargo, m_shooter, m_hood, m_robot), true);
 
    // // launch pad shot
    // new Button(m_operatorController::getXButton)
    //     .whenPressed(new ShootWithSetRPMAndHoodAngle(4000, 32, m_cargo, m_shooter, m_hood, m_robot), true);

    //Using this for debugging and tuning the hood at the practice field 
    // new Button(m_operatorController::getRightBumper)
    // .whileActiveOnce(new ShootManualAdjustRpmAndAngle(() -> m_shootingRpm, () -> m_hoodAngle, m_cargo, m_shooter, m_hood, m_robot), true);

    // new Button(m_operatorController::getBackButton)
    //   .whenPressed(new InstantCommand(() -> m_shootingRpm -= 50));

    // new Button(m_operatorController::getStartButton)
    //   .whenPressed(new InstantCommand(() -> m_shootingRpm += 50));

    // new Button(() -> m_operatorController.getLeftTriggerAxis() >= 0.05)
    //   .whenPressed(new InstantCommand(() -> m_hoodAngle -= 0.5));

    // new Button(() -> m_operatorController.getRightTriggerAxis() >= 0.05)
    //   .whenPressed(new InstantCommand(() -> m_hoodAngle += 0.5));

    // new Button(m_operatorController::getAButton)
    //   .whenPressed(() -> m_hood.setAngleDegrees(18.6), m_hood);

    //   new Button(m_operatorController::getXButton)
    //   .whenPressed(() -> m_hood.setAngleDegrees(23.5), m_hood);

    //   new Button(m_operatorController::getYButton)
    //   .whenPressed(() -> m_hood.setAngleDegrees(27.5), m_hood);

    //   new Button(m_operatorController::getBButton)
    //   .whenPressed(() -> m_hood.setAngleDegrees(33), m_hood);

    //   new Button(m_operatorController::getStartButton)
    //   .whenPressed(() -> m_hood.setAngleDegrees(15), m_hood);


    // new Button(() ->  (m_operatorController.getLeftTriggerAxis() > 0.05))
    //   .whileHeld(new CargoRunIndexer(m_cargo));
     

    // new Button(m_operatorController::getLeftStickButtonPressed)
    //   .whileHeld(new CargoReverseCommand(m_cargoSubsystem, m_intake));

    // ******************* Climb Controls ****************************

    new Button(m_climbController::getStartButton)
      .whenPressed(new InstantCommand(() -> m_elevator.zeroHeight(), m_elevator));
 
    new Button(m_climbController::getBackButton)
      .whileHeld(new InstantCommand(() -> m_arm.zeroEncoder(), m_arm));

    // new Button(m_climbController::getAButton)
    //   .whenPressed(new ElevatorGoToMaxHeight(m_elevator,drivetrain).andThen(new ControllerRumbleCommand(m_climbController, 0.2)));

    // new Button(m_climbController::getBButton)
    //   .whenPressed(new InstantCommand(() -> m_intake.deployIntake()));

    // new Button(m_climbController::getYButton)
    //   .whenPressed(new ClimbMidAuto(m_elevator, m_arm, m_climbController)
    //   .withInterrupt(m_climbController::getBButton));

    //   new Button(m_climbController::getXButton)
    //   .whenPressed(new ClimbMidToHigh(m_elevator, m_arm, m_climbController)
    //   .withInterrupt(m_climbController::getBButton));

    //   new Button(m_climbController::getStartButton)
    //   .whenPressed(new ClimbHighToTraverse(m_elevator, m_arm, m_climbController)
    //   .withInterrupt(m_climbController::getBButton));

    //  new Button(m_climbController::getRightBumper)
    //    .whenPressed(new ClimbFullCommand(m_elevator, m_arm, m_climbController)
    //    .withInterrupt(m_climbController::getBButton));

    //  new Button(m_climbController::getYButton)
    //    .whenPressed(new ClimbHighFull(m_elevator, m_arm, m_climbController)
    //    .withInterrupt(m_climbController::getBButton));

    

   // Rest of climb controls are in the default arm and default elevator commands
  }
  
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }


  //TODO: check if deadband value needs to be changed  
  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  // method that checks if either joystick is active (used to interrupt the dodge commands)
  public Boolean joystickMoving() {
    if(modifyAxis(m_controller.getLeftY()) > 0.0 || modifyAxis(m_controller.getLeftX()) > 0.0){
      return true;
    } 
    return false;
  }

  public void updateManualShooterSettings() {
    SmartDashboard.putNumber("A MANUAL SHOOTING RPM", m_shootingRpm);
    SmartDashboard.putNumber("A MANUAL SHOOTING HOOD ANGLE", m_hoodAngle);
  }

}



