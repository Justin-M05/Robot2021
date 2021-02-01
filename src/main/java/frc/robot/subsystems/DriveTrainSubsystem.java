// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class DriveTrainSubsystem extends SubsystemBase {
  private final WPI_VictorSPX leftMotorOne = new WPI_VictorSPX(Constants.LEFT_MOTOR_ID_ONE);
  private final WPI_VictorSPX leftMotorTwo = new WPI_VictorSPX(Constants.LEFT_MOTOR_ID_TWO);
  private final WPI_VictorSPX rightMotorOne = new WPI_VictorSPX(Constants.RIGHT_MOTOR_ID_ONE);
  private final WPI_VictorSPX rightMotorTwo = new WPI_VictorSPX(Constants.RIGHT_MOTOR_ID_TWO);
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.LEFT_MASTER_ID);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.RIGHT_MASTER_ID);
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMotorOne, leftMotorTwo);
  private final SpeedControllerGroup leftMotorsFinal = new SpeedControllerGroup(leftMotors, leftMaster);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMotorOne, rightMotorTwo);
  private final SpeedControllerGroup rightMotorsFinal = new SpeedControllerGroup(rightMotors, rightMaster);
  private final DifferentialDrive drive = new DifferentialDrive(leftMotorsFinal, rightMotorsFinal);



  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);

  }
  /** Creates a new DriveTrainSubsystem. */
  



  public DriveTrainSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
