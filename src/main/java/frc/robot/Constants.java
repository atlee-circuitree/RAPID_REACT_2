// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;

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
    
    //public static final double frontLeftEncoderOffset = 249.61;
    //public static final double frontRightEncoderOffset = 222.45;
    //public static final double rearLeftEncoderOffset = 94.31;
    //public static final double rearRightEncoderOffset = 121.38;

    public static final double frontLeftEncoderOffset = 285.380859375;
    public static final double frontRightEncoderOffset = 252.685546875;
    public static final double rearLeftEncoderOffset = 130.25390625;
    public static final double rearRightEncoderOffset = 69.345703125;

    public static final int feederMotorPort = 17;
    
    public static final int topShooterMotorPort = 13;
    public static final int bottomShooterMotorPort = 14;

    public static final int lowVelocityTop = 5000;
    public static final int lowVelocityBottom = 5000;

    public static final int mediumVelocityTop = 7700;
    public static final int mediumVelocityBottom = 8000;

    public static final int highVelocityTop = 11200;
    public static final int highVelocityBottom = 7800;

    public static final int turretMotorPort = 19;

    public static final int hookMotorPort = 18;
 
    public static final double trackwidth = 22.5;
    public static final double wheelbase = 22.5;

    //Distance from center of robot to any module
    public static final double drivetrainRadius = Math.sqrt(Math.pow(trackwidth, 2) + Math.pow(wheelbase, 2)); 

    public static final int xboxControllerPort = 0;

    //Drive PIDs
    public static final double rotPID_P = 1;
    public static final double rotPID_I = 0.00;
    public static final double rotPID_D = 0.00;
    public static final double rotPIDMinValue = 0.07;

    //Auto PIDs
    public static final double kPXAutoController = 0.067365;
    public static final double kPYAutoController = 0.067365;
    //public static final double kPXAutoController = 0.15;
    //public static final double kPYAutoController = 0.15;
    public static final double kPAutoThetaController = .5;
    //Was 1

    //Pneumatics
    
    public static final int climbLeftPnumaticDeploy = 10;
    public static final int climbLeftPnumaticRetract = 15;
    public static final int climbRightPnumaticDeploy = 0;
    public static final int climbRightPnumaticRetract = 1;
    public static final int shootPnumaticDeploy = 2;
    public static final int shootPnumaticRetract = 3;
    public static final int kickoutPnumaticDeploy = 9;
    public static final int kickoutPnumaticRetract = 14;


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
