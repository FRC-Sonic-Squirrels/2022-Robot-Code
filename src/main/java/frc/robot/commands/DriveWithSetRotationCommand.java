// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class implements field centric swerve drive, with fixed rotational control. The robot
 * defaults to zero degree rotation. Pressing the Xbox POV buttons change the target angle.
 * 
 * Inspired by Team 1684's comprehensive whitepaper on Swerve.
 * https://www.first1684.com/uploads/2/0/1/6/20161347/chimiswerve_whitepaper__2_.pdf and Team 2910's
 * 2021 competition robot code https://github.com/FRCTeam2910/2021CompetitionRobot
 */

public class DriveWithSetRotationCommand extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  // input suppliers from joysticks
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationPOVSupplier;

  // robot rotation in radians to hold while driving
  private double m_setRotationRadians = 0.0;

  // PID controller to maintain fixed rotation.
  // use a ProfiledPIDController w/ Trapezoidal Profile 
  // https://github.com/wpilibsuite/allwpilib/blob/2ad2d2ca9628ab4130135949c7cea3f71fd5d5b6/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/SwerveModule.java#L27-L34
  private ProfiledPIDController rotationController = new ProfiledPIDController(1.5, 0.0, 0.0,
      new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DrivetrainSubsystem.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

  /**
   * 
   * DriveWithSetRotationCommand - field centric swerve drive, while holding a given robot rotation
   * 
   * @param drivetrainSubsystem
   * @param translationXSupplier
   * @param translationYSupplier
   * @param rotationRadians
   */
  public DriveWithSetRotationCommand(DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
      DoubleSupplier rotationPOVSupplier, double rotationRadians) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;
    m_rotationPOVSupplier = rotationPOVSupplier;
    m_setRotationRadians = rotationRadians;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(0.1, 0.1);  // about 0.1 radians = 6 degrees, 6 deg/sec

    SmartDashboard.putNumber("TargetAngle", Math.toDegrees(m_setRotationRadians));
    SmartDashboard.putNumber("RobotAngleRadians",
        m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
    SmartDashboard.putNumber("RobotAngleVelRadians",
        m_drivetrainSubsystem.getGyroscopeRotationVelocity().getRadians());
    SmartDashboard.putNumber("RotationOutput", 0.0);
    SmartDashboard.putNumber("ThetaVError", rotationController.getVelocityError());
    SmartDashboard.putNumber("ThetaError", rotationController.getPositionError());
    SmartDashboard.putBoolean("ThetaAtTarget", rotationController.atGoal());
    SmartDashboard.putNumber("X", m_translationXSupplier.getAsDouble());
    SmartDashboard.putNumber("Y", m_translationYSupplier.getAsDouble());

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
    rotationController.setGoal(m_setRotationRadians);

    SmartDashboard.putNumber("DriveWithRationAngle", m_setRotationRadians);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double pov = m_rotationPOVSupplier.getAsDouble();
    if (pov >= 0.0) {
      // pov of -1 indicates no POV button pressed

      // POV increases clockwise, so need to negate. Up is forward (0 deg).
      // left (counter clockwise) is positive, right (clockwise) is negative
      if (pov > 180) {
        // example: 270 becomes 90
        pov = 360 - pov;
      } else {
        // example: 90 becomes -90
        pov = -pov;
      }

      if (Math.toRadians(pov) != m_setRotationRadians) {
        // only reset PDI if target changes
        SmartDashboard.putNumber("TargetAngle", pov);

        m_setRotationRadians = Math.toRadians(pov);
      }
    }

    double rotationOutput = rotationController
        .calculate(m_drivetrainSubsystem.getGyroscopeRotation().getRadians(), m_setRotationRadians);

    SmartDashboard.putNumber("TargetAngle", Math.toDegrees(m_setRotationRadians));
    SmartDashboard.putNumber("RobotAngleRadians",
        m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
    SmartDashboard.putNumber("RobotAngleVelRadians",
        m_drivetrainSubsystem.getGyroscopeRotationVelocity().getRadians());
    SmartDashboard.putNumber("RotationOutput", rotationOutput);
    SmartDashboard.putNumber("ThetaVError", rotationController.getVelocityError());
    SmartDashboard.putNumber("ThetaError", rotationController.getPositionError());
    SmartDashboard.putBoolean("ThetaAtTarget", rotationController.atGoal());
    SmartDashboard.putNumber("X", m_translationXSupplier.getAsDouble());
    SmartDashboard.putNumber("Y", m_translationYSupplier.getAsDouble());

    if (Math.abs(rotationOutput) < 0.05) {
      rotationOutput = 0.0;
    }

    m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), rotationOutput,
        m_drivetrainSubsystem.getGyroscopeRotation()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
