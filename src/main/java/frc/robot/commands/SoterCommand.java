// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.SoterSubsystem;
import frc.demacia.utils.Controller.CommandController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SoterCommand extends Command {
  /** Creates a new SoterCommand. */

  SoterSubsystem Soter;

  double DistanceToTarget;
  double Angle;
  double VelocitySotter;
  double verbosityMotor;
  boolean Shoot;

  public SoterCommand(SoterSubsystem Soter) {
    this.Soter = Soter;
    addRequirements(Soter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DistanceToTarget = Soter.CalcolateDistanceToTarget(Constants.TARGET_POSE2D, null);
    Angle = Soter.CalcolateAngle(0, Constants.GEAVITY, Constants.TARGET_HEIGHT, DistanceToTarget);
    VelocitySotter = Soter.calclateVelocitySoter(DistanceToTarget, Constants.TARGET_HEIGHT, Angle, Constants.GEAVITY, Constants.TARGET_HEIGHT);
    verbosityMotor = Soter.calclateVelocityMotor(Constants.wheelDiameter, VelocitySotter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Soter.CalcolateDistanceToTarget(Constants.TARGET_POSE2D, null);
    Soter.CalcolateAngle(VelocitySotter, Constants.GEAVITY, Constants.TARGET_HEIGHT, DistanceToTarget);
    Soter.calclateVelocitySoter(DistanceToTarget, Constants.TARGET_HEIGHT, Angle, Constants.GEAVITY, Constants.TARGET_HEIGHT);
    Soter.calclateVelocityMotor(Constants.wheelDiameter, VelocitySotter);
    Soter.setSoerAngle(Angle);
    if(Shoot == true){
      Soter.shot(verbosityMotor);
    } else {
      Soter.stopShoting();
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
}
