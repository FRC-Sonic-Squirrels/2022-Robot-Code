// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.team2930.lib.Limelight;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private Limelight limelight; 
  private double targetHeight_meters = 0.0;
  private double limeLightHeight_meters = 0.0;
  private double limeLightAngle_degrees = 0.0;
  private double distance_meters = 0.0;

  /** Creates a new Limelight. */
  public LimelightSubsystem(double targetHeight_meters, double limeLightHeight_meters, double limeLightAngle_degrees) {
    this.targetHeight_meters = targetHeight_meters;
    this.limeLightHeight_meters = limeLightHeight_meters;
    this.limeLightAngle_degrees = limeLightAngle_degrees;
    this.limelight = new Limelight();
  }

  @Override
  public void periodic() {
    if (limelight.seesTarget()) {
      distance_meters = limelight.getDist(targetHeight_meters, limeLightHeight_meters , limeLightAngle_degrees);
      SmartDashboard.putNumber("distance ft", Units.metersToFeet(distance_meters));
    }
  }

  /** Returns the distance in meters */
  public double hubDistanceMeters () {
    return distance_meters;
  }
}
