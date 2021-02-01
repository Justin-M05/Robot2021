// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlyWheelSubsystem extends SubsystemBase {

  private final CANSparkMax flyWheelMaster = new CANSparkMax(Constants.FLY_WHEEL_MASTER, MotorType.kBrushless);
  private final CANSparkMax flyWheelSlave = new CANSparkMax(Constants.FLY_WHEEL_SLAVE, MotorType.kBrushless);
  private final SpeedControllerGroup flyWheels = new SpeedControllerGroup(flyWheelMaster, flyWheelSlave);

  public void flyWheelSpeed(double speed) {
    flyWheels.set(speed);
  }
 


  /** Creates a new FlyWheelSubsystem. */
  public FlyWheelSubsystem() {
    flyWheelSlave.restoreFactoryDefaults();
    flyWheelMaster.restoreFactoryDefaults();
    flyWheelSlave.follow(flyWheelMaster, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
