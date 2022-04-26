// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MusicCommands;

import com.team2930.lib.OrchestraMusicController;
import com.team2930.lib.OrchestraMusicController.Songs;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class OrchestraPlayQueueCommand extends CommandBase {
  /** Creates a new OrchestraPlayQueueCommand. */
  String[] m_songFilepaths;
  OrchestraMusicController m_musicController;
  int m_songIndex = 0;


  /**
   * Requires the song queue 
   * requires all the subsystems because we dont want the motors getting mixed signals
   * 
   * @param songFilepaths
   * @param allSubsystems
   */
  public OrchestraPlayQueueCommand(OrchestraMusicController musicController, String[] songFilepaths, Subsystem[] allSubsystems) {
    m_songFilepaths = songFilepaths;
    m_musicController = musicController;

    addRequirements(allSubsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_musicController.isPlaying()){
      m_musicController.playSong(m_songFilepaths[m_songIndex]);
      m_songIndex++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_songIndex = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_songIndex == m_songFilepaths.length-1 && !m_musicController.isPlaying());
  }
}
