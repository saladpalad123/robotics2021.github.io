// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.LEFT_MASTER_ID);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.RIGHT_MASTER_ID);
  private final WPI_TalonFX rightSlave = new WPI_TalonFX(Constants.RIGHT_SLAVE_ID);
  private final WPI_TalonFX leftSlave = new WPI_TalonFX(Constants.LEFT_SLAVE_ID);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final ADXRS450_Gyro gyro  = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  public DriveSubsystem() {
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

  }

  public void arcadeDrive(double fwd, double rot){
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public void gyroAngle(){
    gyro.getAngle();
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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
