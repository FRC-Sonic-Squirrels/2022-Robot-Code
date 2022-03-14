// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final Robot m_robot;

  public final CargoSubsystem m_cargo;
  public final ShooterSubsystem m_shooter;
  public ElevatorSubsystem m_elevator;
  public ArmSubsystem m_arm;

  // The robot's subsystems and commands are defined here...
  public final Drivetrain drivetrain;
  public final IntakeSubsystem m_intake;
  public LimelightSubsystem m_limelight;

  public final XboxController m_controller = new XboxController(0);
  public final XboxController m_operatorController = new XboxController(1);
  public final XboxController m_climbController = new XboxController(2);

  public final SendableChooser<Command> chooser = new SendableChooser<>();
  
  public DriverStation.Alliance m_alliance = DriverStation.getAlliance();

  private UsbCamera camera;
  private boolean climbSubsystemsEnabled = false;

  public void robotInitAddSubsystems() {
    if (!climbSubsystemsEnabled) {
      m_elevator = new ElevatorSubsystem();
      m_arm = new ArmSubsystem();
      m_limelight = new LimelightSubsystem(drivetrain);

      // Configure the button bindings
      configureButtonBindings();

      climbSubsystemsEnabled = true;
    }
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {

    m_robot = robot;

    m_cargo = new CargoSubsystem();
    m_shooter = new ShooterSubsystem(m_robot);
    m_intake = new IntakeSubsystem();
    drivetrain = new Drivetrain();

    SmartDashboard.putData("Auto Mode", chooser);

    // add the new auton trajectories to the auton trajectory chooser
    SwerveTrajectoryAutonomousCommandFactory auton =
        new SwerveTrajectoryAutonomousCommandFactory(drivetrain, m_shooter, m_cargo, m_intake,
            m_robot, Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);

    Command 
    autonOne = auton.twoBallAuto(StartPoseConstants.BLUE_DEF_TOP, FieldConstants.BLUE_CARGO_7);

    chooser.addOption("Auton 1: shoot and move", autonOne);
    chooser.addOption("Auton 2: move and shoot 2", auton.twoBallAutoShoot2(StartPoseConstants.BLUE_DEF_TOP, FieldConstants.BLUE_CARGO_7));
    chooser.addOption("Auton 3: move, shoot 2, push", auton.twoBallAutoShoot2push (StartPoseConstants.BLUE_DEF_TOP, FieldConstants.BLUE_CARGO_7));

    if (m_robot.isReal()) {
      // Creates UsbCamera and sets resolution
      camera = CameraServer.startAutomaticCapture();
      camera.setResolution(320, 240);
      camera.setFPS(20);
    }

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

    new Button(m_controller::getRightTriggerAxis)
            .togglewhenActive(new IntakeCommand(m_intake, m_cargo), true);

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
       .toggleWhenPressed(new IntakeDeployCommand(m_intake, m_cargo));

    new Button(m_operatorController::getYButton)
       .whileHeld(new IntakeReverseCommand(m_intake, m_cargo));

    // 2000 RPM is good for 5 feet
    new Button(m_operatorController::getXButton)
       .whileHeld(new ShootWithSetRPMCommand(3200, m_cargo, m_shooter, m_robot), true);

    // 3000 RPM is good for 10 feet
    new Button(m_operatorController::getBButton)
       .whileHeld(new ShootWithSetRPMCommand(3400, m_cargo, m_shooter, m_robot), true);

    // 1500 RPM is perfecto for right against the hub
    new Button(m_operatorController::getRightBumper)
     .whileActiveOnce(new ShootWithSetRPMCommand(2800, m_cargo, m_shooter, m_robot), true);

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

}



