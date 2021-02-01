// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Shoot extends CommandBase {
  private final FlyWheelSubsystem flyWheelSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private Timer timer = new Timer();
  /** Creates a new Shoot. */
  

  public Shoot(FlyWheelSubsystem flyWheelSubsystem,IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
    this.flyWheelSubsystem = flyWheelSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(flyWheelSubsystem);
    addRequirements(indexerSubsystem);
    addRequirements(intakeSubsystem);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    flyWheelSubsystem.flyWheelSpeed(0.5);
    if (timer.get() >= 2) {
      indexerSubsystem.indexerSpeed(-1);
    }
    else {
      indexerSubsystem.indexerSpeed(0);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    flyWheelSubsystem.flyWheelSpeed(0);
    indexerSubsystem.indexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
