// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2930.lib;

import java.util.ArrayList;
import java.util.Collection;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.util.WPILibVersion;

/** Add your docs here. */
public class OrchestraMusicController extends Orchestra{
  private double secondsPlayed;

  public enum Songs{
    testSong("filepath");

    public final String filepath;

    Songs(String filepath){
      this.filepath = filepath;
    }

  }

  public enum Playlists{
    testPlaylist(Songs.testSong.filepath, "filepath");

    public final String[] files;

    Playlists(String... files){
      this.files = files;
    }

  }

  public OrchestraMusicController(Collection<TalonFX> instruments) {
    super(instruments);
  }

  public static OrchestraMusicController OrchestraFromWpiTalonControllers(Collection<WPI_TalonFX> wpi_TalonFXs){
    ArrayList<TalonFX> baseTalonFXs = new ArrayList<TalonFX>(wpi_TalonFXs);

    return new OrchestraMusicController(baseTalonFXs);
  }


  public void playTestSong(){
    this.playSong(Songs.testSong.filepath);
  }


  public void playSong(String filePath){
    super.stop();
    super.loadMusic(filePath);
    super.play();
  }

  @Override
  public ErrorCode clearInstruments() {
    // TODO Auto-generated method stub
    return super.clearInstruments();
  }
}

