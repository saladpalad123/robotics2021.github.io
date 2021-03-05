// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonSRX shooterMaster = new TalonSRX (CanIdConstants.SHOOTER_MASTER_ID);
  private final TalonSRX shooterSlave = new TalonSRX (CanIdConstants.SHOOTER_SLAVE_ID);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterSlave.follow(shooterMaster);
  }

  public void setSpeed(double speed) {
    shooterMaster.set(ControlMode.PercentOutput, speed);
    //shooterSlave.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
