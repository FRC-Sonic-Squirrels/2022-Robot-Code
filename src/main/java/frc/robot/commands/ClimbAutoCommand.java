// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.SortedMap;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.AddressableLED;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;
import frc.robot.Constants.AutoClimbConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.AutoClimbConstants.Stage_1;
import frc.robot.Constants.AutoClimbConstants.Stage_2;
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

  XboxController m_controller;

  public ClimbAutoCommand(Supplier<Double> POVSupplier, ElevatorSubsystem elevator, ArmSubsystem arm, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_arm = arm;

    m_currentStage = stage.AUTO_0;
    m_currentCommand = getCommandForStage(stage.AUTO_0);

    m_controller = controller;
  
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

    for(int i=0; i<stages.length; i++){
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

  private void rumbleController(){
    m_controller.setRumble(RumbleType.kLeftRumble, 0.5);
    m_controller.setRumble(RumbleType.kRightRumble, 0.5);
  }

  private void stopRumbleController(){
    m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
    m_controller.setRumble(RumbleType.kRightRumble, 0.0);
  }

  //TODO: find appropriate button for this
  private boolean isConfirmButtonPressed(){
    return m_controller.getBackButtonPressed();
  }
  /**
   * rumbles to tell the operator the program is ready to do an action and is seeking approval.
   * This is to have a button press before doing any action with elevator heights or arm rotations
   * 
   * @return command
   */
  private Command getButtonConfirmationCommand(){
    return new SequentialCommandGroup(
      new InstantCommand(() -> rumbleController()),
      new WaitUntilCommand(() -> isConfirmButtonPressed()),
      new InstantCommand(() -> stopRumbleController())
    );
  }

  private Command getStage_0Command(){
    return new InstantCommand();
  }

  private Command getStage_1Command(){
    //this command assumes that the elevator is extended on the mid bar and the robot is ready to be pulled
    return new SequentialCommandGroup(
      getButtonConfirmationCommand(),

      new InstantCommand(() -> m_arm.setArmToSpecificAngle(Stage_1.ARM_TARGET_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle(Stage_1.ARM_TARGET_ANGLE)),

      getButtonConfirmationCommand(),              
      
      new InstantCommand(()-> m_elevator.setElevatorHeight(Stage_1.ELEVATOR_PULL_HEIGHT), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight()),

      getButtonConfirmationCommand(),

      new InstantCommand(() -> m_arm.setArmToSpecificAngle(0), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle(0)),

      getButtonConfirmationCommand(),

      //maybe find a way to bring it to this height slowly so its a smoother transition?
      //we might have to hold the arm angle here if the robot wants to naturally tip
      new InstantCommand(() -> m_elevator.setElevatorHeight(Stage_1.ELEVATOR_SWITCH_TO_ARM_HEIGHT), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight())

      //how do we tell the operator this section is finished? rumble? but thats used for confirming actions? 
      //use smartdashboard to send a boolean ready for next stage? 
    );
  }

  private Command getStage_2Command(){
    return new SequentialCommandGroup(
      getButtonConfirmationCommand(),
      
      new InstantCommand(() -> m_arm.holdAngle(Stage_2.ARM_STARTING_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle(Stage_2.ARM_STARTING_ANGLE)),

      getButtonConfirmationCommand(),

      new InstantCommand(() -> m_elevator.setElevatorHeight(Stage_2.ELEVATOR_EXTENSION_HEIGHT), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight()),

      //have a button confirmation for this? it is shifting the arm angle so technically there should be a button press here 
      getButtonConfirmationCommand(),

      new InstantCommand(() -> m_arm.holdAngle(Stage_2.ARM_HOLD_ANGLE), m_arm),

      getButtonConfirmationCommand(),

      new ParallelCommandGroup(
        new InstantCommand(() -> m_elevator.setElevatorHeight(Stage_2.ELEVATOR_PULL_HEIGHT), m_elevator),
        new InstantCommand(() -> m_arm.setMotorCoastMode()),
        new WaitUntilCommand(() -> m_elevator.isAtHeight())
      ),

      new InstantCommand(() -> m_arm.setMotorBreakMode()),

      getButtonConfirmationCommand(),

      new InstantCommand(() -> m_elevator.setElevatorHeight(Stage_2.ELEVATOR_BRING_ARM_TO_OTHER_SIDE_HEIGHT), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight()),

      getButtonConfirmationCommand(),

      new InstantCommand(() -> m_arm.setArmToSpecificAngle(Stage_2.ARM_BRING_AROUND_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle(Stage_2.ARM_BRING_AROUND_ANGLE)),

      getButtonConfirmationCommand(),

      new InstantCommand(() -> m_elevator.setElevatorHeight(Stage_2.ELEVATOR_PULL_TO_SWITCH_TO_ARM_HEIGHT), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight()),

      getButtonConfirmationCommand(),

      new InstantCommand(() -> m_arm.setArmToSpecificAngle(0), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle(0)),

      getButtonConfirmationCommand(),

      new InstantCommand(() -> m_elevator.setElevatorHeight(Stage_2.ELEVATOR_LIFT_TO_SWITCH_TO_ARM_HEIGHT), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight())

      //same as stage 1 use smartdashboard? 
  
    );
  }

  private Command getStage_3Command(){
    return null;
  }

  private Command getStage_4Command(){
    return null;
  }
}
