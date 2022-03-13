// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.time.Instant;
import java.util.List;
import com.team2930.lib.Limelight;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.StartPoseConstants;
import frc.robot.commands.ArmManualControlCommand;
import frc.robot.commands.CargoReverseCommand;
import frc.robot.commands.DriveFieldCentricCommand;
import frc.robot.commands.DriveWithSetRotationCommand;
import frc.robot.commands.ElevatorControlCommand;
import frc.robot.commands.ShootWithSetRPMCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.LimelightRotateToHubAndShoot;
import frc.robot.commands.ShootCargoCommand;
import frc.robot.commands.DriveHubCentricCommand;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.autonomous.SimpleAutonCommandOne;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final CargoSubsystem m_cargo = new CargoSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();

  // The robot's subsystems and commands are defined here...
  public final Drivetrain drivetrain = new Drivetrain();
  public final IntakeSubsystem m_intake = new IntakeSubsystem(drivetrain);
  public final LimelightSubsystem m_limelight = new LimelightSubsystem(drivetrain);

  //public final VisionSubsystem m_visionSubsystem = new VisionSubsystem(drivetrain);
  public final Robot m_robot;

  public final XboxController m_controller = new XboxController(0);
  public final XboxController m_operatorController = new XboxController(1);
  public final XboxController m_climbController = new XboxController(2);

  public final SendableChooser<Command> chooser = new SendableChooser<>();
  //public final SendableChooser<Command> autonTrajectoryChooser = new SendableChooser<>();
  //public final SendableChooser<Command> simpleTrajectoryChooser = new SendableChooser<>();

  //public final SendableChooser<Pose2d> startPoseChooser = new SendableChooser<>();
  //public final SendableChooser<Command> autonTrajectoryChooser = new SendableChooser<>();
  //public final SendableChooser<Command> simpleAutonChooser = new SendableChooser<>();
  
  public DriverStation.Alliance m_alliance = DriverStation.getAlliance();

  private UsbCamera camera;

  TestTrajectories m_tt = new TestTrajectories(3.5, 2.5, drivetrain, true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {

    m_robot = robot;
    
    // set the starting position of the robot on the field
    // startPoseChooser.addOption("1m left of hub", Constants.ROBOT_1M_LEFT_OF_HUB);
    // startPoseChooser.addOption("blue 1", StartPoseConstants.BLUE_20_13);
    // startPoseChooser.addOption("blue 2", StartPoseConstants.BLUE_22_19);
    // startPoseChooser.addOption("blue 3", StartPoseConstants.BLUE_22_8);
    // startPoseChooser.addOption("blue 4", StartPoseConstants.BLUE_27_6); //note: cannot be used in paths starting with ball 3
    // startPoseChooser.addOption("red 1", StartPoseConstants.RED_27_21);
    // startPoseChooser.addOption("red 2", StartPoseConstants.RED_31_14);
    // startPoseChooser.addOption("red 3", StartPoseConstants.RED_32_19);
    // startPoseChooser.addOption("red 4", StartPoseConstants.RED_32_8);
   

    SwerveTrajectoryFollowCommandFactory.addTestTrajectoriesToChooser(chooser, 1.0, 0.75, drivetrain, true, m_shooter,
        m_cargo, m_intake, m_robot);

    chooser.addOption("Auton shoot and pick up test", shootAndDriveAuton());
    SmartDashboard.putData("Auto Mode", chooser);

    // add the new auton trajectories to the auton trajectory chooser
    SwerveTrajectoryAutonomousCommandFactory auton = new SwerveTrajectoryAutonomousCommandFactory(
        drivetrain, m_shooter, m_cargo, m_intake, m_robot, 1, 0.75);

    Command fourBallAuton_blue = auton.fourBallAutonCommand(StartPoseConstants.BLUE_DEF_BOTTOM, FieldConstants.BLUE_CARGO_3,
        FieldConstants.BLUE_CARGO_2, FieldConstants.BLUE_CARGO_1);
    Command fourBallAuton_red = auton.fourBallAutonCommand(StartPoseConstants.RED_DEF_TOP, FieldConstants.RED_CARGO_3,
        FieldConstants.RED_CARGO_2, FieldConstants.RED_CARGO_1);

    Command threeBallAuton_blue = auton.threeBallAutonCommand(StartPoseConstants.BLUE_DEF_BOTTOM,
        FieldConstants.BLUE_CARGO_3, FieldConstants.BLUE_CARGO_2);
    Command threeBallAuton_red = auton.threeBallAutonCommand(StartPoseConstants.RED_DEF_TOP,
        FieldConstants.RED_CARGO_3, FieldConstants.RED_CARGO_2);

    Command twoBallAuton_blue = auton.twoBallAutonCommand(StartPoseConstants.BLUE_DEF_TOP, FieldConstants.BLUE_CARGO_7);
    Command twoBallAuton_red = auton.twoBallAutonCommand(StartPoseConstants.RED_DEF_BOTTOM, FieldConstants.RED_CARGO_7);

    Command complimentaryAuton_blue = auton.complimentaryAutonCommand("blue");
    Command complimentaryAuton_red = auton.complimentaryAutonCommand("blue");

    Command fiveBallAuton_blue = auton.fiveBallAutonCommand("blue");
    Command fiveBallAuton_red = auton.fiveBallAutonCommand("blue");

    Command 
    autonOne = auton.testAutonCommand(StartPoseConstants.BLUE_DEF_TOP, FieldConstants.BLUE_CARGO_7);

    // autonTrajectoryChooser.addOption("blue 4 ball", fourBallAuton_blue);
    // autonTrajectoryChooser.addOption("blue 3 ball", threeBallAuton_blue);
    // autonTrajectoryChooser.addOption("blue 2 ball", twoBallAuton_blue);
    // autonTrajectoryChooser.addOption("blue helper auton", complimentaryAuton_blue);
    // autonTrajectoryChooser.addOption("blue 5 ball", fiveBallAuton_blue);

    // autonTrajectoryChooser.addOption("red 4 ball", fourBallAuton_red);
    // autonTrajectoryChooser.addOption("red 3 ball", threeBallAuton_red);
    // autonTrajectoryChooser.addOption("red 2 ball", twoBallAuton_red);
    // autonTrajectoryChooser.addOption("red helper auton", complimentaryAuton_red);
    // autonTrajectoryChooser.addOption("red 5 ball", fiveBallAuton_red);

    chooser.addOption("Auton 1: shoot and move", autonOne);

//    SmartDashboard.putData("auton commands", autonTrajectoryChooser);

    if (m_robot.isReal()) {
      // Creates UsbCamera and sets resolution
      camera = CameraServer.startAutomaticCapture();
      camera.setResolution(320, 240);
      camera.setFPS(20);
    }
 

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // drivetrain.setDefaultCommand(new DefaultDriveCommand(
    //   drivetrain,
    //   () -> -modifyAxis(m_controller.getLeftY() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
    //   () -> -modifyAxis(m_controller.getLeftX() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) ,
    //   () -> -modifyAxis(m_controller.getRightX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 4)
    // ));
    
    drivetrain.setDefaultCommand(new DriveFieldCentricCommand(
      drivetrain, 
      () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 
      () -> -modifyAxis(m_controller.getRightX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));

    m_elevator.setDefaultCommand(new ElevatorControlCommand(m_elevator, m_climbController,
        Constants.ElevatorConstants.elevatorSpeedMultiplier));
    m_arm.setDefaultCommand(new ArmManualControlCommand(m_arm, m_climbController, 0.3));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {

    //-------------- DRIVER CONTROLS DEFINED HERE --------------------------  

    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(drivetrain::resetFieldCentric);

    new Button(m_controller::getXButton)
            .whenPressed(new DriveHubCentricCommand(drivetrain, 
            () -> -modifyAxis(m_controller.getRightX()), 
            () -> -modifyAxis(m_controller.getLeftY())));

    new Button(m_controller::getYButton)
            .whenPressed(new DriveWithSetRotationCommand(drivetrain,
            () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> m_controller.getPOV(), 0.0));

    new Button(m_controller::getBButton)
            .whenPressed(new DriveRobotCentricCommand(drivetrain,
            () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.8, 
            () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.8,
            () -> -modifyAxis(m_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5));

    new Button(m_controller::getAButton)
            .whenPressed(new DriveFieldCentricCommand(drivetrain,
            () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 
            () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    new Button(m_controller::getLeftBumper)
            .whileHeld(new LimelightRotateToHubAndShoot(2000, m_limelight, drivetrain, m_cargo, m_shooter, m_intake, m_robot));

    // new Button(m_controller::getLeftBumper)
    //   .whileHeld(new VisionRotateToCargo(m_visionSubsystem, drivetrain));

    // new Button(m_controller::getRightBumper)
    //   .whileHeld(new VisionDriveToCargo(m_visionSubsystem, drivetrain));

    // **************** OPERATOR CONTROLS ********************************

    // //shoot 1 ball idk if this is useful but its here 
    // new Button(m_operatorController::getBButton)
    //   .whileHeld(new ShootOneCargoCommand(m_cargoSubsystem, m_shooterSubsystem, m_intake));
 
    //shoot while holding 
    //new Button(m_operatorController::getRightBumper)
    //  .whileHeld(new ShootCargoCommand(m_cargoSubsystem, m_shooterSubsystem, m_intake, m_robot));

    //Deploy intake while holding 
    new Button(m_operatorController::getAButton)
       .whileHeld(new IntakeDeployCommand(m_intake, m_cargo));

    new Button(m_operatorController::getYButton)
       .whileHeld(new IntakeReverseCommand(m_intake, m_cargo));

    // 2000 RPM is good for 5 feet
    new Button(m_operatorController::getXButton)
       .whileHeld(new ShootWithSetRPMCommand(2500, m_cargo, m_shooter, m_robot));

    // 3000 RPM is good for 10 feet
    new Button(m_operatorController::getBButton)
       .whileHeld(new ShootWithSetRPMCommand(3000, m_cargo, m_shooter, m_robot));

    // 1500 RPM is perfecto for right against the hub
    new Button(m_operatorController::getRightBumper)
     .whileHeld(new ShootWithSetRPMCommand(1500, m_cargo, m_shooter, m_robot));

    // new Button(m_operatorController::getLeftStickButtonPressed)
    //   .whileHeld(new CargoReverseCommand(m_cargoSubsystem, m_intake));

    // ******************* Climb Controls ****************************

    new Button(m_climbController::getStartButton)
      .whenPressed(new InstantCommand(() -> m_elevator.zeroHeight(), m_elevator));
 
    new Button(m_climbController::getBackButton)
      .whileHeld(new InstantCommand(() -> m_arm.zeroEncoder(), m_arm));

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
  private static double modifyAxis(double value) {
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

  public Command shootAndDriveAuton() {

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_shooter.setFlywheelRPM(1500), m_shooter),
      new WaitCommand(2),
      //might cause problems in cargo transition 
      new ShootWithSetRPMCommand(1500, m_cargo, m_shooter, m_robot)
        .withTimeout(3),
        SwerveTrajectoryFollowCommandFactory.straightForward2mCommand(m_tt, drivetrain)
    );
    //   new WaitCommand(0.5),
    //   new ParallelCommandGroup(
    //     SwerveTrajectoryFollowCommandFactory.straightForward2mCommand(m_tt, drivetrain),
    //     new IntakeDeployCommand(m_intake, m_cargoSubsystem) 
    //       .withTimeout(4) //deploy or run? 
    //   ),
    //   new ParallelCommandGroup(
    //      SwerveTrajectoryFollowCommandFactory.straightBack1mCommand(m_tt, drivetrain),
    //      new InstantCommand(() -> m_shooterSubsystem.setFlywheelRPM(1500), m_shooterSubsystem)
    //   ),
    //   new ShootWithSetRPMCommand(1500, m_cargoSubsystem, m_shooterSubsystem, m_intake, m_robot)
    //     .withTimeout(2)
    // );
  }

  public TrajectoryConfig getTrajectoryConfig() {
    //TODO: THESE ARE HARD CODED VALUES
    TrajectoryConfig config = new TrajectoryConfig(1, 0.75)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(drivetrain.kinematics());

   
      // Limits the velocity of the robot around turns such that no wheel of a swerve-drive robot
      // goes over a specified maximum velocity.
      SwerveDriveKinematicsConstraint swerveConstraint = new SwerveDriveKinematicsConstraint(
          drivetrain.kinematics(), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
      config.addConstraint(swerveConstraint);
    
    return config;
  }
}



