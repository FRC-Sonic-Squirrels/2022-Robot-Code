// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.StartPoseConstants;
import frc.robot.commands.ArmManualControlCommand;
import frc.robot.commands.CargoReverseCommand;
import frc.robot.commands.ClimbManualCommand;
import frc.robot.commands.DriveFieldCentricCommand;
import frc.robot.commands.DriveWithSetRotationCommand;
import frc.robot.commands.ElevatorControlCommand;
import frc.robot.commands.ShootOneCargoCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.ShootCargoCommand;
import frc.robot.commands.VisionDriveToCargo;
import frc.robot.commands.VisionRotateToCargo;
import frc.robot.commands.DriveHubCentricCommand;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain drivetrain = new Drivetrain();
  //public final ArmSubsystem m_arm = new ArmSubsystem();
  //public final VisionSubsystem m_visionSubsystem = new VisionSubsystem(drivetrain);
  public final CargoSubsystem m_cargoSubsystem = new CargoSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem(drivetrain);
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final Robot m_robot;

  public final XboxController m_controller = new XboxController(0);
  public final XboxController m_operatorController = new XboxController(1);

  public final SendableChooser<Command> chooser = new SendableChooser<>();
  public final SendableChooser<Pose2d> startPoseChooser = new SendableChooser<>();
  public final SendableChooser<Command> autonTrajectoryChooser = new SendableChooser<>();

  public DriverStation.Alliance m_alliance = DriverStation.getAlliance();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {

    m_robot = robot;
    
    // set the starting position of the robot on the field
    startPoseChooser.addOption("1m left of hub", Constants.ROBOT_1M_LEFT_OF_HUB);
    startPoseChooser.addOption("blue 1", StartPoseConstants.BLUE_20_13);
    startPoseChooser.addOption("blue 2", StartPoseConstants.BLUE_22_19);
    startPoseChooser.addOption("blue 3", StartPoseConstants.BLUE_22_8);
    startPoseChooser.addOption("blue 4", StartPoseConstants.BLUE_27_6); //note: cannot be used in paths starting with ball 3
    startPoseChooser.addOption("red 1", StartPoseConstants.RED_27_21);
    startPoseChooser.addOption("red 2", StartPoseConstants.RED_31_14);
    startPoseChooser.addOption("red 3", StartPoseConstants.RED_32_19);
    startPoseChooser.addOption("red 4", StartPoseConstants.RED_32_8);

    drivetrain.setGyroscopeHeadingDegrees(0);
    drivetrain.setPose(Constants.ROBOT_1M_LEFT_OF_HUB, drivetrain.getGyroscopeRotation());

    // SwerveTrajectoryFollowCommandFactory.addTestTrajectoriesToChooser(chooser, 1.0, 0.75, drivetrain, true, m_shooterSubsystem,
    //     m_cargoSubsystem, m_intake, m_robot);
    // SmartDashboard.putData("Auto Mode (discontinued)", chooser);

    // // TODO: figure out if getSelected() will work properly or just return null
    // SwerveTrajectoryAutonomousCommandFactory.addAutonTrajectoriesToChooser(autonTrajectoryChooser, 1.0, 0.75,
    //     startPoseChooser.getSelected(), drivetrain, true, m_shooterSubsystem, m_cargoSubsystem, m_intake, m_robot);
    // SmartDashboard.putData("Auto Mode (real)", autonTrajectoryChooser);

    // Creates UsbCamera and MjpegServer [1] and connects them
    // UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    // MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    // mjpegServer1.setSource(usbCamera);

    // // Creates the CvSink and connects it to the UsbCamera
    // CvSink cvSink = new CvSink("opencv_USB Camera 0");
    // cvSink.setSource(usbCamera);

    // // Creates the CvSource and MjpegServer [2] and connects them
    // CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    // MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    // mjpegServer2.setSource(outputStream);
    

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
    
    //m_arm.setDefaultCommand(new InstantCommand());


    //m_elevator.setDefaultCommand(new InstantCommand());
    //control winch with right joystick 
    // m_armSubsystem.setDefaultCommand(new InstantCommand(
    //   () -> m_armSubsystem.setArmPercentOutput(modifyAxis(m_operatorController.getRightTriggerAxis())), 
    //   m_armSubsystem));
    
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
            .whenPressed(drivetrain::zeroGyroscope);

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

    // new Button(m_controller::getLeftBumper)
    //   .whileHeld(new VisionRotateToCargo(m_visionSubsystem, drivetrain));

    // new Button(m_controller::getRightBumper)
    //   .whileHeld(new VisionDriveToCargo(m_visionSubsystem, drivetrain));

    // -------------- OPERATOR CONTROLS DEFINED HERE -------------------------- 

    //shoot 1 ball idk if this is useful but its here 
    new Button(m_operatorController::getBButton)
      .whileHeld(new ShootOneCargoCommand(m_cargoSubsystem, m_shooterSubsystem, m_intake));
 
    //shoot while holding 
    new Button(m_operatorController::getRightBumper)
      .whileHeld(new ShootCargoCommand(m_cargoSubsystem, m_shooterSubsystem, m_intake, m_robot));

    //toggle climbing mode 
    new Button(m_operatorController::getLeftBumper)
      .whenPressed(new ClimbManualCommand(m_arm, m_elevator, m_operatorController));

    //Deploy intake while holding 
    new Button(m_operatorController::getAButton)
      .whileHeld(new IntakeDeployCommand(m_intake, m_cargoSubsystem));

    //reverse intake and indexer while holding
    new Button(m_operatorController::getYButton)
      .whileHeld(new IntakeReverseCommand(m_intake, m_cargoSubsystem));

    new Button(m_operatorController::getStartButton)
       .whenPressed(new InstantCommand(() -> m_elevator.zeroHeight(), m_elevator));

    new Button(m_operatorController::getBackButton)
      .whenPressed(new InstantCommand(() -> m_arm.zeroEncoder(), m_arm));
    
    // new Button(m_operatorController::getRightBumper)
    //   .whileHeld(new ArmManualControlCommand(() -> m_operatorController.getRightY(), m_arm));

    // new Button(m_operatorController::getLeftBumper)
    //   .whileHeld(new ElevatorControlCommand(() -> m_controller.getLeftY(), m_elevator));

  

    // //rotate arm one step in the positive direction (towards the front of robot)
    // new Button(m_operatorController::getBackButton)
    //   .whenPressed(new InstantCommand(() -> m_arm.incrementArmAngle(1), m_arm));

    // // rotate arm one step in the negative direction (towards the back of robot)
    // new Button(m_operatorController::getStartButton)
    //   .whenPressed(new InstantCommand(() -> m_arm.incrementArmAngle(-1), m_arm));

    // //this exists but are we gonna use it? 
    // new Button(m_operatorController::getXButton)
    //   .whileHeld(new CargoReverseCommand(m_cargoSubsystem, m_intake));
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
    value = deadband(value, 0.08);

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



