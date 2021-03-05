// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(CanIdConstants.LEFT_MASTER_ID);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(CanIdConstants.RIGHT_MASTER_ID);
  private final WPI_TalonFX rightSlave = new WPI_TalonFX(CanIdConstants.RIGHT_SLAVE_ID);
  private final WPI_TalonFX leftSlave = new WPI_TalonFX(CanIdConstants.LEFT_SLAVE_ID);

  private final ADXRS450_Gyro gyro  = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(gyro.getRotation2d());

  private DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(TrajectoryConstants.SIMPLE_MOTOR_FEED_FOWARD, TrajectoryConstants.DRIVE_KINEMATICS, 10);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration
  (true, 40, 60, 1);

  public final List<Trajectory> pathList = new ArrayList<>();

  private final PIDController ramseteController = new PIDController(TrajectoryConstants.KP, 0, 0);

  TrajectoryConfig config = new TrajectoryConfig
  (TrajectoryConstants.MAX_VELOCITY, TrajectoryConstants.MAX_ACCELERATION)
  .setKinematics(TrajectoryConstants.DRIVE_KINEMATICS)
  .addConstraint(autoVoltageConstraint);

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

    pathList.add(
        0,
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 0)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config));

    pathList.add(
        1,
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(2, 0)),
            new Pose2d(4, 0, new Rotation2d(0)),
            config));


  }

  public void voltDrive(double leftVolts, double rightVolts){
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    differentialDrive.feed();
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

  public void resetPose(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public double getAverageDistance(){
    return ((getLeftWheelPosition() + getRightWheelPosition()) / 2);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
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

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public RamseteCommand ramsete(Trajectory path) {
    return new RamseteCommand(
        path,
        odometry::getPoseMeters,
        new RamseteController(),
        TrajectoryConstants.SIMPLE_MOTOR_FEED_FOWARD,
        TrajectoryConstants.DRIVE_KINEMATICS,
        this::getWheelSpeeds,
        ramseteController,
        ramseteController,
        this::voltDrive,
        this);
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
