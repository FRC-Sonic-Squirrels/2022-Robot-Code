// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// code based off RI3D's beam balance command:
// https://github.com/GOFIRST-Robotics/Ri3D-2023/blob/6d79b376bde95481b32f0a98edbf424580653960/src/main/java/frc/robot/commands/BalanceOnBeamCommand.java

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

public class AutoEngage extends CommandBase {

  private Drivetrain drivetrain;

  private Timer timeEngaged = new Timer();
  private double error;
  private double currentPitch;
  private double drivePower;
  private boolean flip;

  private double kP;
  private double balancedThresholdDegrees = 4.5;
  // private TunableNumber timeRequiredBalanced =
  //     new TunableNumber("AutoEngage/timeRequiredBalanced", 1);
  private double maxPowerPercent = 1;

  private DoubleSupplier y_supplier;
  /** Creates a new AutoEngage. */
  public AutoEngage(Drivetrain drivetrain, double kP, boolean flip) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.flip = flip;
    this.kP = kP;
    // y_supplier = yAxisSup;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // currentPitch = -1 * -modifyAxis(y_supplier.getAsDouble()) * 45;

    this.currentPitch = drivetrain.getGyroscopePitch();

    error = currentPitch;
    drivePower = (kP * error);

    // drivePower = Math.copySign(drivePower, error);

    if (flip) {
      drivePower *= -1;
    }
    // The robot I referenced when making this needed extra power while in reverse.
    //  // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    //  if (drivePower < 0) {
    //    drivePower *= DrivetrainConstants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    //  }

    // Limit the max power
    if (Math.abs(drivePower)
        > Constants.AutoConstants.maxVelocity * maxPowerPercent) {
      drivePower =
          Math.copySign(
            Constants.AutoConstants.maxVelocity * maxPowerPercent,
              drivePower);
    }

      

    // Starts the timer when we are within the specified threshold of being 'flat' (gyroscope pitch
    // of 0 degrees)
    if (Math.abs(error) <= balancedThresholdDegrees) {
      timeEngaged.start();
    } else {
      timeEngaged.reset();
    }

    if (timeEngaged.get() >= 0.3) {
      drivetrain.setXStance();
      drivetrain.drive(0, 0, 0);
    } else {
      drivetrain.drive(drivePower, 0, 0);
      // drivetrain.disableXstance();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setXStance();
    // the wheels stay at their last known rotation, even if this command ends the wheels will point
    // x stance until driver gives input. We want to disable x stance so that the driver can regain
    // control in teleop
    // drivetrain.enableFieldRelative();
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (timeEngaged.get()>timeRequiredBalanced.get());
    return Math.abs(error) <= 3;
  }
}
