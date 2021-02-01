// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class Shoot extends CommandBase {
  private final XboxController controller;
  private final FlyWheelSubsystem flyWheelSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  /** Creates a new Shoot. */
  

  public Shoot(FlyWheelSubsystem flyWheelSubsystem, XboxController controller, IndexerSubsystem indexerSubsystem) {
    this.controller = controller;
    this.flyWheelSubsystem = flyWheelSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(flyWheelSubsystem);
    addRequirements(indexerSubsystem);
  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.button.a.isPressed == true) {
      flyWheelSubsystem.flyWheelSpeed(0.5);

      robot.timer.start;
      if (robot.timer >= 2) {
        indexerSubsystem.indexerSpeed(-0.2);
      }
      else {
        indexerSubsystem.indexerSpeed(0);
      }
    } else {
      flyWheelSubsystem.flyWheelSpeed(0);
      indexerSubsystem.indexerSpeed(0);
      robot.timer.reset;
      
    }

  }


  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
