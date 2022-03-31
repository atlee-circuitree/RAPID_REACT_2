// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TurretSubsystem;

public class ControlClimbPistons extends CommandBase {
   
  private final Pneumatics pnuematic;
  private boolean up; 

  public ControlClimbPistons(boolean isThisGoingUp, Pneumatics ps) {
 
    pnuematic = ps;
    up = isThisGoingUp;
    addRequirements(pnuematic);

  }
 
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (up == true) {

    pnuematic.climbPistonsUp();

    } else {

    pnuematic.climbPistonsDown();

    }
 
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