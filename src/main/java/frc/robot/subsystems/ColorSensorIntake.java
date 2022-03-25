// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensorIntake extends SubsystemBase {

  // Use I2C on MXP, the onboard I2C can cause the Rio to lock up
  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private String opponentColor = "";

  // TODO: determine exact CMY and RGB colors for cargo.
  private final Color kBlueTarget = new Color(0.15, 0.4, 0.5);
  private final Color kRedTarget = new Color(0.5, 0.32, 0.1);
  private final Color kNothing = new Color(0.32, 0.44, 0.3);
  private final Color kYellow = new Color(0.27, 0.6, 0.11);

  public ColorSensorIntake() {

    //m_colorMatcher.setConfidenceThreshold(0.8);

    DriverStation.Alliance alliance = DriverStation.getAlliance();

    if (alliance == DriverStation.Alliance.Blue) {
      opponentColor = "Red";
    } else {
      opponentColor = "Blue";
    }

    m_colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes16bit,
        ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain6x);


    // colors we want to match
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kNothing);
    m_colorMatcher.addColorMatch(kYellow);

    SmartDashboard.putNumber("ColorSensor Red", 0.0);
    SmartDashboard.putNumber("ColorSensor Green", 0.0);
    SmartDashboard.putNumber("ColorSensor Blue", 0.0);
    SmartDashboard.putNumber("ColorSensor Confidence", 0.0);
    SmartDashboard.putString("ColorSensor Detected Color", "Initializing");
  }

  @Override
  public void periodic() {
    senseCargoColor();
  }

  public boolean opponentCargoDetected() {
    if (senseCargoColor() == opponentColor) {
      return true;
    }
    return false;
  }

  // TODO: What happens if there is no cargo in front of the sensor?
  /**
   * Run cargo sensor, red or blue
   */
  public String senseCargoColor() {
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
    } else if (match.color == kYellow) {
      colorString = "Yellow";
    } else if (match.color == kNothing) {
      colorString = "Nothing";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("ColorSensor Red", detectedColor.red);
    SmartDashboard.putNumber("ColorSensor Green", detectedColor.green);
    SmartDashboard.putNumber("ColorSensor Blue", detectedColor.blue);
    SmartDashboard.putNumber("ColorSensor Confidence", match.confidence);
    SmartDashboard.putString("ColorSensor Detected Color", colorString);

    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);

    return colorString;
  }
}
