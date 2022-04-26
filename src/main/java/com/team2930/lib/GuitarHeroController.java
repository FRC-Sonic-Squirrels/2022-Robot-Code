// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2930.lib;

import edu.wpi.first.wpilibj.GenericHID;

/** Add your docs here. */
public class GuitarHeroController extends GenericHID {
  public enum Button {
    greenButton(0), redButton(0), yellowButton(0), blueButton(0), orangeButton(0),

    select(0), start(0),

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

  public boolean getRedButton(){
    return getRawButtonPressed(Button.redButton.value);
  }

  public double getTrebleAxis(){
    return getRawAxis(Axis.treble.value);
  }

  


  
}
