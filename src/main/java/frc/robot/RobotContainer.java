// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Operate;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private final XboxController controller = new XboxController(Constants.XBOX_CONTROLLER);
  private final DriveTrainSubsystem drive = new DriveTrainSubsystem();
  private final TeleopDrive teleopDrive = new TeleopDrive(drive, controller);
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final FlyWheelSubsystem flyWheel = new FlyWheelSubsystem();
  private final Operate operate = new Operate(flyWheel, controller, indexer, intake);
  

  
  

  // The robot's subsystems and commands are defined here...


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drive.setDefaultCommand(teleopDrive);
    indexer.setDefaultCommand(operate);
    intake.setDefaultCommand(operate);
    flyWheel.setDefaultCommand(operate);
    
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(new Shoot(flyWheel, indexer, intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    return null ;
  }

  
  public XboxController getController() {
    return controller;
  }
}
