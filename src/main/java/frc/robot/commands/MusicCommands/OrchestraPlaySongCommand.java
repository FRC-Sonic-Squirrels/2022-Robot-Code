// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MusicCommands;

import com.team2930.lib.OrchestraMusicController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class OrchestraPlaySongCommand extends CommandBase {
  /** Creates a new OrchestraPlaySongCommand. */
  OrchestraMusicController m_musicController;
  String m_filepath;

  public OrchestraPlaySongCommand(OrchestraMusicController musicController, String songFilepath, Subsystem[] allSubsystems) {
    m_musicController = musicController;
    m_filepath = songFilepath;


    addRequirements(allSubsystems);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_musicController.playSong(m_filepath);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
