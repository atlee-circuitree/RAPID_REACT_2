// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.regex.MatchResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterInAuto extends CommandBase {
   
  private final TurretSubsystem turret;
  private final Pneumatics pneumatic;
  private final LimeLightSubsystem limelight;
  private final FeederSubsystem feeder;
  private double velocity;
  private double bottomVelocity;
  private double distance;
  private long timeout = 1000;
  Timer shooterTime = new Timer();

  double startingTime = shooterTime.get();   

  public ShooterInAuto(double targetVelocity, double targetBottom, TurretSubsystem ts, Pneumatics ps, LimeLightSubsystem ls, FeederSubsystem fs) {
 
    turret = ts;
    pneumatic = ps;
    limelight = ls;
    feeder = fs;
    bottomVelocity = targetBottom;
    velocity = targetVelocity;
    addRequirements(turret);
    
  }
 
  @Override
  public void initialize() {
    
    //Limelight stuff
    distance = limelight.getDistanceToTarget();
    //PUT ALGORITHIM HERE
    shooterTime.start();
    shooterTime.reset();
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
      turret.runTurretWithVelocity(velocity, bottomVelocity);
      Timer.delay(1);
      pneumatic.shooterUp();
      Timer.delay(1.5);
      pneumatic.shooterDown();
      Timer.delay(1);
      turret.runTurretWithVelocity(0, bottomVelocity);

    
  }

  @Override
  public void end(boolean interrupted) {

     
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if  (shooterTime.get() > 1.25) {

      turret.runTurretWithVelocity(0, 1.4);
      return true;

    } else {
      return false;

    } 
 
  }
}
