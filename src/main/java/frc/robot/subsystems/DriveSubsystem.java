// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.SPI;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(CanIdConstants.LEFT_MASTER_ID);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(CanIdConstants.RIGHT_MASTER_ID);
  private final WPI_TalonFX rightSlave = new WPI_TalonFX(CanIdConstants.RIGHT_SLAVE_ID);
  private final WPI_TalonFX leftSlave = new WPI_TalonFX(CanIdConstants.LEFT_SLAVE_ID);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration
  (true, 40, 60, 1);

  private final ADXRS450_Gyro gyro  = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  public DriveSubsystem() {
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.configSupplyCurrentLimit(currentLimit);
    rightMaster.configSupplyCurrentLimit(currentLimit);
    leftSlave.configSupplyCurrentLimit(currentLimit);
    rightSlave.configSupplyCurrentLimit(currentLimit);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    resetEncoders();
    resetGyro();

  }

  public void arcadeDrive(double fwd, double rot){
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public void gyroAngle(){
    gyro.getAngle();
  }

  public double getHeading(){
    return gyro.getRotation2d().getDegrees();
  }

  public void calibrateGyro(){
    gyro.calibrate();
  }

  public void resetGyro(){
    gyro.reset();
  }

  public void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
  }

  public double getAverageDistance(){
    return ((getLeftWheelPosition() + getRightWheelPosition()) / 2);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftWheelSpeed(), getRightWheelSpeed());
  }

  private double getLeftWheelPosition(){
    return (leftMaster.getSelectedSensorPosition() * DriveConstants.WHEEL_CIRCUMFERENCE_METERS/ 
    DriveConstants.TALONFX_ENCODER_CPR)
    / DriveConstants.GEAR_RATIO;
  }

  private double getRightWheelPosition(){
    return (rightMaster.getSelectedSensorPosition() * DriveConstants.WHEEL_CIRCUMFERENCE_METERS/ 
    DriveConstants.TALONFX_ENCODER_CPR)
    / DriveConstants.GEAR_RATIO;
  }

  private double getLeftWheelSpeed(){
    return leftMaster.getSelectedSensorVelocity(0) * 10 / DriveConstants.TALONFX_ENCODER_CPR / 
    DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
  }

  private double getRightWheelSpeed(){
    return rightMaster.getSelectedSensorVelocity(0) * 10 / DriveConstants.TALONFX_ENCODER_CPR / 
    DriveConstants.GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
