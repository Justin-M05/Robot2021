// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private final WPI_TalonSRX indexerMotor = new WPI_TalonSRX(Constants.INDEX_MOTOR);
  private final DigitalInput sensorIntake = new DigitalInput(Constants.SENSOR_INTAKE);
  private final DigitalInput sensorSpacer = new DigitalInput(Constants.SENSOR_SPACER);
  private final DigitalInput sensorFull = new DigitalInput(Constants.SENSOR_FULL);

  public void indexerAuto() {
    if (sensorFull.get() == true) {
      if ((sensorIntake.get() == false && sensorSpacer.get() == false) || (sensorIntake.get() == true && sensorSpacer.get() == false) || (sensorIntake.get() == false && sensorSpacer.get() == true)) {
        indexerMotor.set(Constants.INDEXER_SPEED);
      }
      else {
        indexerMotor.set(0);
      }
    } 
    else {
      indexerMotor.set(0);
    }
  }



  public void indexerSpeed(double speed) {
    indexerMotor.set(speed);
  }

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
