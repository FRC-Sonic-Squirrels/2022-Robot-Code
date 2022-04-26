// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2930.lib;

import edu.wpi.first.wpilibj.GenericHID;

/** Add your docs here. */
public class GuitarHeroController extends GenericHID {
  public enum Button {
    greenButton(0), redButton(0), yellowButton(0), blueButton(0), orangeButton(0),

    selectButton(0), startButton(0),

    switchUp(0), switchDown(0),

    motionSensor(0);

    public final int value;

    Button(int value) {
      this.value = value;
    }

    public String toString() {
      return this.name();
    }
  }

  public enum Axis {
    treble(0);

    @SuppressWarnings("MemberName")
    public final int value;

    Axis(int value) {
      this.value = value;
    }

    public String toString() {
      return this.name();
    }
  }

  public GuitarHeroController(int port) {
    super(port);
    // TODO Auto-generated constructor stub
  }

  public boolean getGreenButton(){
    return getRawButtonPressed(Button.greenButton.value);
  }

  public boolean getRedButton(){
    return getRawButtonPressed(Button.redButton.value);
  }

  public boolean getYellowButton(){
    return getRawButtonPressed(Button.yellowButton.value);
  }

  public boolean getBlueButton(){
    return getRawButtonPressed(Button.blueButton.value);
  }

  public boolean getOrangeButton(){
    return getRawButtonPressed(Button.orangeButton.value);
  }

  public boolean getSelectButton(){
    return getRawButtonPressed(Button.selectButton.value);
  }

  public boolean getStartButton(){
    return getRawButtonPressed(Button.startButton.value);
  }

  public boolean getSwitchUp(){
    return getRawButtonPressed(Button.switchUp.value);
  }

  public boolean getSwitchDown(){
    return getRawButtonPressed(Button.switchDown.value);
  }

  public boolean getMotionSensor(){
    return getRawButton(Button.motionSensor.value);
  }



  public double getTrebleAxis(){
    return getRawAxis(Axis.treble.value);
  }
}
