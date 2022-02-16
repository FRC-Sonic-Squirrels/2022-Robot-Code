// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

/** Add your docs here. */
public class SimulatorTest extends DifferentialDrivetrainSim {

  private DCMotor m_driveMotor;
  private double m_gearing;
  private double m_jKgMetersSquared;
  private double m_massKg;
  private double m_wheelRadiusMeters;
  private double m_trackWidthMeters;
  private Matrix<N7, N1> m_measurementStdDevs;

  public SimulatorTest(DCMotor driveMotor, double gearing, double jKgMetersSquared, double massKg,
      double wheelRadiusMeters, double trackWidthMeters, Matrix<N7, N1> measurementStdDevs) {

    super(driveMotor, gearing, jKgMetersSquared, massKg, wheelRadiusMeters, trackWidthMeters,
        measurementStdDevs);
    // m_driveMotor = driveMotor;
    // m_gearing = gearing;
    // m_jKgMetersSquared = jKgMetersSquared;
    // m_massKg = massKg;
    // m_wheelRadiusMeters = wheelRadiusMeters;
    // m_trackWidthMeters = trackWidthMeters;
    // m_measurementStdDevs = measurementStdDevs;

  }

  

}
