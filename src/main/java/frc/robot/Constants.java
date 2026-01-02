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

import edu.wpi.first.math.geometry.Pose2d;
import frc.demacia.utils.Motors.TalonFXConfig;
import frc.demacia.utils.Sensors.DigitalEncoderConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

public final class Constants {

    public static final double MAX_VELOCITY = 0;
    public static final double LOW_VELOCITY = 0;

    public static final Pose2d TARGET_POSE2D = new Pose2d(0, 0,  null);

    public static final int SOTER_MOTOR_ID = 15;
    public static final Canbus SOTER_MOTOR_CANBUS = Canbus.Rio;
    public static final String SOTER_MOTOR_NAME = "Soter Motor";

    public static final double KP_SOTER = 0;
    public static final double KI_SOTER = 0;
    public static final double KD_SOTER = 0;
    public static final double KS_SOTER = 0;
    public static final double KV_SOTER = 0;
    public static final double KA_SOTER = 0;
    public static final double KG_SOTER = 0;

    public static final double MAX_VELOCITY_SOTER = 0;
    public static final double MAX_ACCELERATION_SOTER = 0;
    public static final double MAX_JERK_SOTER = 0; 

    public static final double GEARE_RATIO_SOTER = 0; 


    public static final TalonFXConfig SOTER_MOTOR_CONFIG = new TalonFXConfig(SOTER_MOTOR_ID, SOTER_MOTOR_CANBUS, SOTER_MOTOR_NAME)
    .withBrake(true)
    .withPID(KP_SOTER,KI_SOTER,KD_SOTER,KS_SOTER,KV_SOTER,KA_SOTER,KG_SOTER)
    .withMotionParam(MAX_VELOCITY_SOTER, MAX_ACCELERATION_SOTER, MAX_JERK_SOTER)
    .withInvert(false)
    .withRadiansMotor(GEARE_RATIO_SOTER);

    public static final int SOTER_ANGLE_MOTOR_ID = 16;
    public static final Canbus SOTER_ANGLE_MOTOR_CANBUS = Canbus.Rio;
    public static final String SOTER_ANGLE_MOTOR_NAME = "Soter Angle Motor";

    public static final double KP_ANGLE = 0;
    public static final double KI_ANGLE = 0;
    public static final double KD_ANGLE = 0;
    public static final double KS_ANGLE = 0;
    public static final double KV_ANGLE = 0;
    public static final double KA_ANGLE = 0;
    public static final double KG_ANGLE = 0;

    public static final double MAX_VELOCITY_ANGLE = 0;
    public static final double MAX_ACCELERATION_ANGLE = 0;
    public static final double MAX_JERK_ANGLE = 0;

    public static final double GEARE_RATIO_ANGLE = 0;

    public static final TalonFXConfig SOTER_ANGLE_MOTOR_CONFIG = new TalonFXConfig(SOTER_ANGLE_MOTOR_ID, SOTER_ANGLE_MOTOR_CANBUS, SOTER_ANGLE_MOTOR_NAME)
    .withBrake(true)
    .withPID(KP_ANGLE, KI_ANGLE, KD_ANGLE, KS_ANGLE, KV_ANGLE, KA_ANGLE, KG_ANGLE)
    .withMotionParam(MAX_VELOCITY_ANGLE, MAX_ACCELERATION_ANGLE, MAX_JERK_ANGLE)
    .withRadiansMotor(GEARE_RATIO_ANGLE);
    

    public static final int SOTER_ANGLE_ENCODER_CHANNEL = 0;
    public static final String SOTER_ANGLE_ENCODER_NAME = "Soter Angle Encoder";

    public static final double SOTER_ANGLE_ENCODER_OFFSET = 0;

    public static final DigitalEncoderConfig SOTER_ANGLE_ENCODER_CONFIG = new DigitalEncoderConfig(SOTER_ANGLE_ENCODER_CHANNEL, SOTER_ANGLE_ENCODER_NAME)
    .withInvert(false)
    .withOffset(SOTER_ANGLE_ENCODER_OFFSET);
}
