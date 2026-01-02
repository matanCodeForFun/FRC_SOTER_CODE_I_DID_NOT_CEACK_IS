// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import frc.demacia.utils.Motors.TalonFXMotor;
import frc.demacia.utils.Sensors.DigitalEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SoterSubsystem extends SubsystemBase {
  /** Creates a new SoterSubsystem. */

  TalonFXMotor soterMotor;
  TalonFXMotor soterAngleMotor;
  DigitalEncoder encoder;

  public SoterSubsystem() {
    soterMotor = new TalonFXMotor(Constants.SOTER_MOTOR_CONFIG);
    soterAngleMotor = new TalonFXMotor(Constants.SOTER_ANGLE_MOTOR_CONFIG);
    encoder = new DigitalEncoder(Constants.SOTER_ANGLE_ENCODER_CONFIG);
    soterAngleMotor.setEncoderPosition(getEncoderAngle());
  }

  public double getEncoderAngle(){
    return encoder.get();
  }

  public double getMotorAngle(){
    return soterAngleMotor.getCurrentAngle();
  }

  public void setSoerAngle(double wontedAngle){
    soterAngleMotor.setAngle(wontedAngle);
  }

  public void shot(double wantedVelocity){
    soterMotor.setVelocity(MathUtil.clamp(wantedVelocity, Constants.LOW_VELOCITY, Constants.MAX_VELOCITY));
  }

  public void stopShoting(){
    soterMotor.stopMotor();
  }

  public void setPower(double power){
    soterMotor.setDuty(power);
  }

  public double CalcolateDistanceToTarget(Pose2d target, Pose2d robotPose){
    return Math.sqrt(Math.pow(target.getY() - robotPose.getY(), 2) + Math.pow(target.getX() - robotPose.getX(), 2));
  }

  public double CalcolateAngle(double verbosity, double gravity, double trgetHeight, double DistanceToTarget){
    double root = (Math.sqrt(Math.pow(verbosity,4) - gravity * (gravity * Math.pow(DistanceToTarget,2) + 2 * trgetHeight * Math.pow(verbosity,2))));
    return Math.atan(Math.pow(verbosity,2) + root / (gravity * DistanceToTarget));
  }

  public double calclateVelocitySoter(double DistanceToTarget, double trgetHeight, double angle, double gravity, double TargetHeight){
    return Math.sqrt((gravity * Math.pow(DistanceToTarget, 2) / 2 * Math.pow(Math.cos(angle), 2) * (DistanceToTarget * Math.tan(angle) - TargetHeight)));
  }
    

  public double calclateVelocityMotor(double wheelDiameter, double VelocityInSoter){
    return VelocityInSoter * 60 /Math.PI * wheelDiameter;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
