// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithSetRPMCommand extends CommandBase {
  private CargoSubsystem m_cargoSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private Robot m_robot;
  private long m_time;
  private double m_rpm;

  public ShootWithSetRPMCommand(int flyWheelRPM, CargoSubsystem cargoSubsystem, ShooterSubsystem shooterSubsystem, Robot robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cargoSubsystem = cargoSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_robot = robot;
    m_rpm = flyWheelRPM;
    m_time = 0;


    // drivetrain is not included in the requirements, as it use in a "read only"
    // fashion, to call getPose(). 
    addRequirements(cargoSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //by default from testing on 2/26 2000 works well enough for low goal shots 
    
    //m_rpm = SmartDashboard.getNumber("AAA shooting rpm testing", 2000);
    SmartDashboard.putNumber("SHOOTING RPM", m_rpm);

    m_shooterSubsystem.setFlywheelRPM(m_rpm);
    //m_intakeSubsystem.deployIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wait until flywheel is fully revved
    // once it is, turn on upper cargo belt
    // once upper ball has been released, go back to intake mode
    SmartDashboard.putBoolean("AAA can shoot", false);

    if (m_shooterSubsystem.isAtDesiredRPM()) {
      SmartDashboard.putBoolean("AAA can shoot", true);
      m_cargoSubsystem.setShootMode();
      //m_intakeSubsystem.deployIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_cargoSubsystem.setStopMode();
    //m_intakeSubsystem.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // the command will automatically stop when both cargo are released in autonomous
    // if (m_robot.isAutonomous()) {
    //   // the command will stop .5 seconds after no cargo is detected, to let the cargo finish shooting
    //   if ((! m_cargoSubsystem.cargoInUpperBelts()) && (! m_cargoSubsystem.cargoInLowerBelts())) {
    //     if (m_time == 0) {
    //       m_time = System.currentTimeMillis();
    //     }
    //     if (System.currentTimeMillis() - m_time >= 500) {
    //       return true;
    //     }
    //   }
    // }
    // the command will be manually executed and ended by holding a button in teleop
    return false;
  }
}
