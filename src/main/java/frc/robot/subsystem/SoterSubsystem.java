// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import frc.demacia.utils.Motors.TalonFXMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SoterSubsystem extends SubsystemBase {
  /** Creates a new SoterSubsystem. */

  TalonFXMotor soterMotor;

  public SoterSubsystem() {
    soterMotor = new TalonFXMotor(Constants.SOTER_MOTOR_CONFIG);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
