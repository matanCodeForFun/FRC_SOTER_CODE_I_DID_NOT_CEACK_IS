// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

import frc.demacia.utils.Motors.TalonFXConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

public final class Constants {


    public static final int SOTER_MOTOR_ID = 15;
    public static final Canbus SOTER_MOTOR_CANBUS = Canbus.Rio;
    public static final String SOTER_MOTOR_NAME = "Soter Motor";

    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double KG = 0;

    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double MAX_JERK = 0; 

    public static final double GEARE_RATIO = 0; 


    public static final TalonFXConfig SOTER_MOTOR_CONFIG = new TalonFXConfig(SOTER_MOTOR_ID, SOTER_MOTOR_CANBUS, SOTER_MOTOR_NAME)
    .withBrake(true)
    .withPID(KP,KI,KD,KS,KV,KA,KG)
    .withMotionParam(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK)
    .withInvert(false)
    .withRadiansMotor(GEARE_RATIO);


}
