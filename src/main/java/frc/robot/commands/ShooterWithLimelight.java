// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterWithLimelight extends CommandBase {
   
  private final TurretSubsystem turret;
  private final Pneumatics pneumatic;
  private final LimeLightSubsystem limelight;
  private double velocity;
  private double distance;
  private long timeout = 1000;

  public ShooterWithLimelight(double targetVelocity ,TurretSubsystem ts, Pneumatics ps, LimeLightSubsystem ls) {
 
    turret = ts;
    pneumatic = ps;
    limelight = ls;
    velocity = targetVelocity;
    addRequirements(turret);

  }
 
  @Override
  public void initialize() {
    
    //Limelight stuff
    distance = limelight.getDistanceToTarget();
    //PUT ALGORITHIM HERE

    turret.runTurretWithVelocity(velocity);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Timer.delay(1);
    pneumatic.shooterUp();
    Timer.delay(1);
    turret.runTurretWithVelocity(0);
    pneumatic.shooterDown();
    end(true);
 
  }

  @Override
  public void end(boolean interrupted) {
     
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}
