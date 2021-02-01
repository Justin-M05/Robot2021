// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Operate extends CommandBase {
  private final XboxController controller;
  private final FlyWheelSubsystem flyWheelSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  /** Creates a new Operate. */
  public Operate(FlyWheelSubsystem flyWheelSubsystem, XboxController controller, IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.flyWheelSubsystem = flyWheelSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(flyWheelSubsystem);
    addRequirements(indexerSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flyWheelSubsystem.flyWheelSpeed(controller.getTriggerAxis(Hand.kRight));
    if ((controller.getTriggerAxis(Hand.kLeft) <= 0.1) && (controller.getTriggerAxis(Hand.kLeft) >= -0.1))  {
      indexerSubsystem.indexerAuto();
    }
    else {
      indexerSubsystem.indexerSpeed(controller.getTriggerAxis(Hand.kLeft));
    }
    intakeSubsystem.intakeSpeed(controller.getY(Hand.kRight));

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
