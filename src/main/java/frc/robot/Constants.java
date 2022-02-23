// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int frontLeftDrvMotorPort = 4;
    public static final int frontRightDrvMotorPort = 2;
    public static final int rearLeftDrvMotorPort = 6;
    public static final int rearRightDrvMotorPort = 8;

    public static final int frontLeftRotMotorPort = 3;
    public static final int frontRightRotMotorPort = 1;
    public static final int rearLeftRotMotorPort = 5;
    public static final int rearRightRotMotorPort = 7;

    public static final int frontLeftRotEncoderPort = 11;
    public static final int frontRightRotEncoderPort = 9;
    public static final int rearLeftRotEncoderPort = 10;
    public static final int rearRightRotEncoderPort = 12;

    public static final double frontLeftEncoderOffset = 14.45;
    public static final double frontRightEncoderOffset = 359.12;
    public static final double rearLeftEncoderOffset = 16.08;
    public static final double rearRightEncoderOffset = 164.09;

    public static final double trackwidth = 14.5;
    public static final double wheelbase = 25.5;
    //Distance from center of robot to any module
    public static final double drivetrainRadius = Math.sqrt(Math.pow(trackwidth, 2) + Math.pow(wheelbase, 2)); 

    public static final int xboxControllerPort = 0;


    //Instansiated in this order:
    //FrontLeft, FrontRight, RearLeft, RearRight
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelbase / 2, -trackwidth / 2),
            new Translation2d(wheelbase / 2, trackwidth / 2),
            new Translation2d(-wheelbase / 2, -trackwidth / 2),
            new Translation2d(-wheelbase / 2, trackwidth / 2));

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);

}
