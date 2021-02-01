// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
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
  private final XboxController controller = new XboxController(Constants.XBOX_CONTROLLER);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.7101550117116572);


  
  private final AHRS gyro = new AHRS(Port.kMXP);

  private Pose2d m_pPose2d = new Pose2d();

  public double getLeftEncoderPosition() {
    return leftMaster.getSelectedSensorPosition(0);
  }
  public double getRightEncoderPosition() {
    return rightMaster.getSelectedSensorPosition(0);
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d;
  }

  DifferentialDriveOdometry m_oOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  
  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
  }
/*
  public void velocityMode(XboxController controller) {
    var chasisSpeeds = new ChassisSpeeds(3.7 * controller.getY(Hand.kLeft), 0.0, Units.degreesToRadians(360) * controller.getX(Hand.kLeft));
  
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chasisSpeeds);
      // Left velocity
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;

     // Right velocity
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;

  }
*/
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    var chasisSpeeds = new ChassisSpeeds(3.7 * controller.getY(Hand.kLeft), 0.0, Units.degreesToRadians(360) * controller.getX(Hand.kLeft));
  
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chasisSpeeds);
    if (System.currentTimeMillis() % 100 == 0) {
     System.out.println("Left Wheels Speed = " +  wheelSpeeds.leftMetersPerSecond + ", Right Wheels Speed = " + wheelSpeeds.rightMetersPerSecond);
    }
    /*
    var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
    m_pPose2d = m_oOdometry.update(gyroAngle, getLeftEncoderPosition(), getRightEncoderPosition());
    System.out.println("Rotation = " +  m_pPose2d.getRotation() + ", X = " + m_pPose2d.getX() + ", Y = " + m_pPose2d.getY() );
    */
  }
  
}
