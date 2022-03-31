/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
//Limelight Values
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


 
public class LimeLightSubsystem extends SubsystemBase {


  public static String limelightDashboard;

  // private static drivetrainSubsystem m_drivetrainSubsystem = new
  // drivetrainSubsystem();

  boolean m_LimelightHasValidTarget = false;
  double m_LimelightDriveCommand = 0.0;
  double m_LimelightSteerCommand = 0.0;

  // Local variables to store network values
  boolean b_tv = false;
  double dbl_tx, dbl_tv, dbl_ty, dbl_ta, dbl_ts, dbl_thor, dbl_tvert, dbl_tshort, dbl_tlong;

  public boolean HasValidTarget() {
    return b_tv;
  }

  public double VerticalOffset() {
    return dbl_ty;
  }

  public double HorizontalOffset() {
    return dbl_tx;
  }

  public double TargetArea() {
    return dbl_ta;
  }

  public double Skew() {
    return dbl_ts;
  }

  public double BoundingShortSide() {
    return dbl_tshort;
  }

  public double BoundingLongSide() {
    return dbl_tlong;
  }

  public double BoundingHorizontal() {
    return dbl_thor;
  }

  public double BoundingVertical() {
    return dbl_tvert;
  }

  public void ReadNetworkTables() {

    double dbl_tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (dbl_tv > 0) {
      b_tv = true;
    } else {
      b_tv = false;
    }
    dbl_tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    dbl_ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    dbl_ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    dbl_ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    dbl_tshort = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(0);
    dbl_tlong = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(0);
    dbl_thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    dbl_tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
  }

  public LimeLightSubsystem() {
  
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Set LEDs to current pipeline setting
    
  }

  public void EnableLED()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //Set LEDs to current pipeline setting
  }

  public void DisableLED()
  {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Set LEDs to current pipeline setting
  }

  public void updateLimelightTracking() {

    final double STEER_K = 0.05; // how hard to turn toward the target
    final double DRIVE_K = 0.00; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    

    if (tv < 1.0) {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering

    double dead_band = 0.15;
    double steer_cmd = tx * STEER_K;
    if(steer_cmd != 0)
    {
      if(steer_cmd > 0 && steer_cmd < dead_band)
        steer_cmd =dead_band; //dead band
      else if(steer_cmd < 0 && steer_cmd > -dead_band)
        steer_cmd = -dead_band;
    }
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;


    System.out.print("tv: ");
    System.out.print(tv);
    System.out.print(" tx ");
    System.out.print(tx);
    System.out.print(" ty ");
    System.out.print(ty);
    System.out.print(" ta ");
    System.out.print(ta);
    System.out.print(" steer ");
    System.out.print(m_LimelightSteerCommand);
    System.out.print(" drive ");
    System.out.print(m_LimelightDriveCommand);

  }

  public void LimelightSteer() {

    updateLimelightTracking();
     
  }

  public void stopDrive() {

    //m_drivetrainSubsystem.robotDrive.arcadeDrive(0.0, 0.0);

  }

  @Override
  public void periodic() {
    ReadNetworkTables();
    limelightDashboard = "Limelight Horizontal/" + dbl_tx + ";";
    limelightDashboard = limelightDashboard + "Limelight Vertical/" + dbl_ty + ";";
    limelightDashboard = limelightDashboard + "Theoretical Distance To Target/" + getDistanceToTarget() + ";";
    
    SmartDashboard.putNumber("Limelight Distance", getDistanceToTarget());

  }


  public double getDistanceToTarget(){

    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  
    double targetOffsetAngle_Vertical = ty;

    //how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 30.3;

    //distance from the center of the Limelight lens to the floor
    double limelightHeightInches = 26.0;

    //distance from the target to the floor
    double goalHeightInches = 100.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);
    double distanceFromLimelighttoGoalMeters = distanceFromLimelightToGoalInches / 39.37;
    //return in meters
    if (HasValidTarget() == false) {

      return 0;

    } else {

      return distanceFromLimelighttoGoalMeters;

    }

  }
 
}

