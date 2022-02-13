// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.SortedMap;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoClimbConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ClimbAutoCommand extends CommandBase {
  enum stage{
    AUTO_0,
    AUTO_1,
    AUTO_2,
    AUTO_3,
  }

  stage[] stages = {stage.AUTO_0, stage.AUTO_1, stage.AUTO_2, stage.AUTO_3};
  stage m_currentStage;
  stage m_lastStage = stage.AUTO_3;

  Command m_currentCommand;

  ElevatorSubsystem m_elevator;
  ArmSubsystem m_arm;
  Supplier<Double> m_POVSupplier;

  HashMap<stage, Command> stageCommands = new HashMap<stage, Command>();

  public ClimbAutoCommand(Supplier<Double> POVSupplier, ElevatorSubsystem elevator, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_arm = arm;

    m_currentStage = stage.AUTO_0;
    m_currentCommand = getCommandForStage(stage.AUTO_0);

    assignStagesCommands();
    addRequirements(m_elevator, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_currentCommand.isScheduled()){
      if(m_POVSupplier.get()==90){ m_currentStage = getNextStage();} 
      if(m_POVSupplier.get() == 270){ m_currentStage = getPreviousStage();}

      m_currentCommand = getCommandForStage(m_currentStage);
      m_currentCommand.schedule(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private stage getNextStage(){
    if(m_currentStage == m_lastStage){ return stage.AUTO_0;}

    for(int i=0; i<stages.length; i++){
      if(m_currentStage == stages[i]){
        return stages[i+1];
      }
    }
    return null;
  }

  private stage getPreviousStage(){
    if(m_currentStage == stage.AUTO_0){ return stage.AUTO_0; }

    for(int i=stages.length; i>=0; i++){
      if(m_currentStage == stages[i]){
        return stages[i-1];
      }
    }
    return null;
  }

  private Command getCommandForStage(stage stage){
    if(stageCommands.containsKey(stage)) { 
      return stageCommands.get(stage);
    }

    return null;
  }

  //TODO: make sure this gets updated with how many stages we have 
  private void assignStagesCommands(){
    stageCommands.put(stage.AUTO_0, getStage_0Command());
    stageCommands.put(stage.AUTO_1, getStage_1Command());
    stageCommands.put(stage.AUTO_2, getStage_2Command());
    stageCommands.put(stage.AUTO_3, getStage_3Command());
  }

  private Command getStage_0Command(){
    return new InstantCommand();
  }

  private Command getStage_1Command(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        //arm might have to move first before we pull
        new InstantCommand(()-> m_elevator.setElevatorHeight(AutoClimbConstants.Stage_1.ELEVATOR_PULL_HEIGHT_STAGE1), m_elevator),
        new InstantCommand(() -> m_arm.setArmToSpecificAngle(AutoClimbConstants.Stage_1.ARM_TARGET_ANGLE_STAGE1), m_arm)
      ),
      new InstantCommand(() -> m_arm.setArmToSpecificAngle(0), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle(0)),
      //maybe find a way to bring it to this height slowly so its a smoother transition?
      new InstantCommand(() -> m_elevator.setElevatorHeight(AutoClimbConstants.Stage_1.ELEVATOR_SWITCH_TO_ARM_HEIGHT), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight())
    );
  }

  private Command getStage_2Command(){
    return null;
  }

  private Command getStage_3Command(){
    return null;
  }

  private Command getStage_4Command(){
    return null;
  }
}
