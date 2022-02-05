// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DriveFieldCentricCommand;
import frc.robot.commands.DriveWithSetRotationCommand;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.VisionDriveToCargo;
import frc.robot.commands.VisionRotateToCargo;
import frc.robot.commands.DriveHubCentricCommand;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
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
  //public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem(drivetrain);

  public final XboxController m_controller = new XboxController(0);
  public final XboxController m_operatorController = new XboxController(1);

  public final SendableChooser<Command> chooser = new SendableChooser<>();
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // set the starting position of the robot on the field
    // TODO: need a chooser object to select starting position and angle
    drivetrain.setGyroscopeHeadingDegrees(0);
    drivetrain.setPose(Constants.ROBOT_1M_LEFT_OF_HUB, drivetrain.getGyroscopeRotation());

    SwerveTrajectoryFollowCommandFactory.addTestTrajectoriesToChooser(chooser, 1.0, 0.75, drivetrain, true);
    SmartDashboard.putData("Auto mode", chooser);

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
    
    drivetrain.setDefaultCommand(new DriveWithSetRotationCommand(drivetrain,
        () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> m_controller.getPOV(), 0.0));

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
            () -> -modifyAxis(m_controller.getLeftY()) * drivetrain.MAX_VELOCITY_METERS_PER_SECOND *0.8, 
            () -> -modifyAxis(m_controller.getLeftX()) * drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.8,
            () -> -modifyAxis(m_controller.getRightX()) * drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND*0.5));

    new Button(m_controller::getAButton)
      .whileHeld(new VisionRotateToCargo(m_visionSubsystem, drivetrain));

    new Button(m_controller::getRightBumper)
      .whileHeld(new VisionDriveToCargo(m_visionSubsystem, drivetrain));

    
    new Button(m_operatorController::getXButton)
      .whileHeld(new IntakeDeploy(m_intake));

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
}



