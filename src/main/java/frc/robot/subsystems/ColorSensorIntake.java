// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensorIntake extends SubsystemBase {

  // Use I2C on MXP, the onboard I2C can cause the Rio to lock up
  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
 
  
  // TODO: determine exact CMY and RGB colors for cargo. 
  private final Color kBlueTarget = ColorMatch.makeColor(0.0, 0.5, 0.5);
  private final Color kRedTarget = ColorMatch.makeColor(0.5454, 0.0908, 0.36352);
  
  public ColorSensorIntake() {
    // colors we want to match
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    
    SmartDashboard.putNumber("Red", 0.0);
    SmartDashboard.putNumber("Green", 0.0);
    SmartDashboard.putNumber("Blue", 0.0);
    SmartDashboard.putNumber("Confidence", 0.0);
    SmartDashboard.putString("Detected Color", "Initializing");
  }

  
  // TODO: What happens if there is no cargo in front of the sensor? 
  /**
   * Run cargo sensor, red or blue
   */
  public void senseCargoColor() {
    // This method will be called once per scheduler run
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString = "";
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    }
    else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  
    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);
  }
}