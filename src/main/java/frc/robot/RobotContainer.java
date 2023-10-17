// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.function.BooleanSupplier;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.DriveFieldCentricCommand;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.commands.DriveWithSetRotationCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.ShootManualAdjustRpm;
import frc.robot.commands.ShootWithSetRPM;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
  public final Drivetrain drivetrain;
  public final IntakeSubsystem m_intake;

  // Controllers
  public final XboxController m_controller = new XboxController(0);
  // public final XboxController m_operatorController = new XboxController(1);

  public final SendableChooser<Command> chooser = new SendableChooser<>();

  public DriverStation.Alliance m_alliance = DriverStation.getAlliance();

  // public Command climbRumbleCommand = new ControllerClimbMaxHeightRumble(m_climbController,
  // m_elevator);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {

    m_robot = robot;

    m_cargo = new CargoSubsystem();
    m_shooter = new ShooterSubsystem(m_robot, m_robot.revPDH);
    m_intake = new IntakeSubsystem();
    drivetrain = new Drivetrain();

    SmartDashboard.putData("Auto Mode", chooser);

    //Tunable RPMs
    SmartDashboard.putNumber("high node RPM", Constants.ShooterConstants.HIGH_NODE_RPM);
    SmartDashboard.putNumber("mid node RPM", Constants.ShooterConstants.MID_NODE_RPM);

    // add the new auton trajectories to the auton trajectory chooser
    SwerveTrajectoryAutonomousCommandFactory auton =
        new SwerveTrajectoryAutonomousCommandFactory(drivetrain, m_shooter, m_cargo, m_intake,
            m_robot, Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);


    chooser.addOption("nothing", new InstantCommand());

    // chooser.addOption("rightTaxi", auton.rightTaxi());

    // chooser.addOption("leftTaxi", auton.leftTaxi());

    chooser.addOption("sideTaxi", auton.sideTaxi());

    chooser.addOption("scoreHigh", auton.scoreHigh());

    chooser.addOption("Score Low", auton.scoreLow());

    // chooser.addOption("hp2piece", auton.hp2piece());

    // chooser.addOption("hp2pieceEngage", auton.hp2pieceEngage());

    // chooser.addOption("hp3piece", auton.hp3piece());

    // chooser.addOption("wall2piece", auton.wall2piece());

    // chooser.addOption("wall2pieceEngage", auton.wall2pieceEngage());

    // chooser.addOption("wall3piece", auton.wall3piece());

    chooser.addOption("[LOW]middle1pieceEngage", auton.middle1pieceEngage());

    chooser.addOption("[HIGH]middle1pieceEngage", auton.highMiddle1pieceEngage());


    // default auton is to do nothing, for SAFETY
    chooser.setDefaultOption("nothing", new InstantCommand());

    drivetrain.setDefaultCommand(new DriveFieldCentricCommand(drivetrain,
        () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getRightX())
            * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // m_elevator.setDefaultCommand(new ElevatorControlCommand(m_elevator, m_climbController,
    // Constants.ElevatorConstants.elevatorSpeedMultiplier));

    // m_arm.setDefaultCommand(new ArmManualControlCommand(m_arm, m_climbController, 0.3));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {

    // ************************ DRIVER CONTROLS [START] *******************************

    // Back button resets field centric, forward is the current heading
    new Trigger(m_controller::getBackButton)
        // No requirements because we don't need to interrupt anything
        .onTrue(new InstantCommand(drivetrain::resetFieldCentric));

    new Trigger(m_controller::getStartButton)
      .onTrue(Commands.runOnce(() -> drivetrain.setXStance(), drivetrain));

    // robot centric
    new Trigger(m_controller::getYButton).onTrue(new DriveRobotCentricCommand(drivetrain,
        () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
            * 0.8,
        () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
            * 0.8,
        () -> -modifyAxis(m_controller.getRightX())
            * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // field centric
    new Trigger(m_controller::getBButton).onTrue(new DriveFieldCentricCommand(drivetrain,
        () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getRightX())
            * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // rotate 0 degrees
    new Trigger(m_controller::getAButton).whileTrue(new DriveWithSetRotationCommand(drivetrain,
        () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -1, 0));

    // rotate to hp wall
    new Trigger(m_controller::getXButton).whileTrue(new DriveWithSetRotationCommand(drivetrain,
    () -> -modifyAxis(m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -1, Math.toRadians(DriverStation.getAlliance() == Alliance.Blue ? 90 : -90)));

    // deploy intake
    new Trigger(() -> (m_controller.getRightTriggerAxis() > 0.05))
        .whileTrue(new IntakeDeployCommand(m_intake, m_cargo));

    // rumble
    new Trigger(() -> m_cargo.cargoInLowerBelts()).onTrue(
    new SequentialCommandGroup(
    Commands.runOnce( () ->  m_controller.setRumble(RumbleType.kBothRumble, 0.3)),
    Commands.waitSeconds(0.5),
    Commands.runOnce( () ->  m_controller.setRumble(RumbleType.kBothRumble, 0))
    ));

    // reverse intake
    new Trigger(() -> (m_controller.getLeftTriggerAxis() > 0.05))
        .whileTrue(new IntakeReverseCommand(m_intake, m_cargo));

    // mid node
    new Trigger(m_controller::getRightBumper).whileTrue(
        new ShootManualAdjustRpm(() -> SmartDashboard.getNumber("high node RPM", Constants.ShooterConstants.HIGH_NODE_RPM), m_cargo, m_shooter, m_robot));

    // mid node
    new Trigger(m_controller::getLeftBumper).whileTrue(
        new ShootManualAdjustRpm(() -> SmartDashboard.getNumber("mid node RPM", Constants.ShooterConstants.MID_NODE_RPM), m_cargo, m_shooter, m_robot));

    // ************************ DRIVER CONTROLS [END] *******************************

    // **************** OPERATOR CONTROLS [START] ********************************

    

    // **************** OPERATOR CONTROLS [END] ********************************
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


  // TODO: check if deadband value needs to be changed
  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  // method that checks if either joystick is active (used to interrupt the dodge commands)
  public Boolean joystickMoving() {
    if (modifyAxis(m_controller.getLeftY()) > 0.0 || modifyAxis(m_controller.getLeftX()) > 0.0) {
      return true;
    }
    return false;
  }
}


