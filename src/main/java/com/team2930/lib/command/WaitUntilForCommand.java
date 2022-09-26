// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2930.lib.command;

import java.sql.Time;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** Add your docs here. */
public class WaitUntilForCommand extends WaitUntilCommand {
  private double m_lastTime = 0.0;
  private double m_timeToStayTrue;
  private double m_timeConditionHasBeenTrue = 0.0;

  public WaitUntilForCommand(BooleanSupplier condition, double timeToStayTrueSeconds) {
    super(condition);

    m_timeToStayTrue = timeToStayTrueSeconds;
  }


 @Override
 public boolean isFinished() {
   //TODO: remove smartdashboard debug prints
   //if condition is false return false
   SmartDashboard.putBoolean("wait until for debug condition status", super.isFinished());
   if(!super.isFinished()){
     //reset time condition has been true if condition is false
    m_timeConditionHasBeenTrue = 0.0;
    return false;
   }
   
   //update the time the condition has been true for
   m_timeConditionHasBeenTrue = Timer.getFPGATimestamp() - m_lastTime;

   m_lastTime = Timer.getFPGATimestamp();

   SmartDashboard.putNumber("wait until for debug time condition has been true", m_timeConditionHasBeenTrue);
   SmartDashboard.putNumber("wait until for debug currentTime", Timer.getFPGATimestamp());
   SmartDashboard.putNumber("wait until for debug last time", m_lastTime);

   //if the time its been true for is greater than the time required to be 
   //true than the command is finished
   if(m_timeConditionHasBeenTrue >= m_timeToStayTrue){
     return true;
   }

   return false;
 }

}
