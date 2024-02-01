/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick controller;

  private static final int leftDeviceID = 1; 
  private static final int leftDeviceID2 = 3; 
  private static final int rightDeviceID = 2;
  private static final int rightDeviceID2 = 4;

  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor2;

  private Boolean enableFilter = false;
  SlewRateLimiter filterLeft = new SlewRateLimiter(6);
  SlewRateLimiter filterRight = new SlewRateLimiter(6);
  SlewRateLimiter filterTurn = new SlewRateLimiter(6);
  

  private double speedLimit = 1;
  private Boolean tankDrive = true;
  
  AHRS navX;

  @Override
  public void robotInit() {
    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushed);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushed);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushed);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushed);

    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);
    

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    controller = new Joystick(0);

    CameraServer.startAutomaticCapture();

    try {
      navX = new AHRS(SPI.Port.kMXP);
      navX.enableLogging(true);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
  }

  @Override 
  public void robotPeriodic() {

    SmartDashboard.putNumber("Motor 1 Applied Output", m_leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Motor 3 Applied Output", m_leftMotor2.getAppliedOutput());
    
    SmartDashboard.putNumber("Motor 2 Applied Output", m_rightMotor.getAppliedOutput());
    SmartDashboard.putNumber("Motor 4 Applied Output", m_rightMotor2.getAppliedOutput());

    SmartDashboard.putNumber("Speed Limit", speedLimit * 100);
    SmartDashboard.putBoolean("Filter Enabled", enableFilter);

    if (controller.getRawButtonPressed(4)) {
      enableFilter = !enableFilter;
    }
    if (controller.getRawButtonPressed(1)) {
      speedLimit = 0.50;
    }
    if (controller.getRawButtonPressed(2)) {
      speedLimit = 0.75;
    }
    if (controller.getRawButtonPressed(3)) {
      speedLimit = 1;
    }

    if (controller.getRawButtonPressed(9)) {
      tankDrive = !tankDrive;
    }

    navXTelemetry();
  }

  @Override
  public void teleopPeriodic() {
    double leftDrive = controller.getRawAxis(1);
    double rightDrive = controller.getRawAxis(5);
    double turn = controller.getX();
    SmartDashboard.putNumber("Joystick Axis 1", controller.getRawAxis(1));
    SmartDashboard.putNumber("Joystick Axis 5", -controller.getRawAxis(5));

    

    leftDrive *= speedLimit;
    rightDrive *= speedLimit;

     if(enableFilter){
      leftDrive = filterLeft.calculate(leftDrive);
      rightDrive = filterRight.calculate(rightDrive);
      turn = filterTurn.calculate(turn);
    }

    if(tankDrive){
      m_myRobot.tankDrive(leftDrive, -rightDrive);
    }
    else {
      m_myRobot.arcadeDrive(-turn, -leftDrive);
    }

    SmartDashboard.putNumber("Left drive", leftDrive);
    SmartDashboard.putNumber("Right drive", rightDrive);

    
  }

  private void navXTelemetry(){
    boolean zero_yaw_pressed = controller.getTrigger();
    if (zero_yaw_pressed) {
      navX.zeroYaw();
    }

    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("IMU_Connected", navX.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", navX.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", navX.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", navX.getPitch());
    SmartDashboard.putNumber("IMU_Roll", navX.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    SmartDashboard.putNumber("IMU_CompassHeading", navX.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", navX.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

    SmartDashboard.putNumber("IMU_TotalYaw", navX.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", navX.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    SmartDashboard.putNumber("IMU_Accel_X", navX.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", navX.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", navX.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", navX.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    SmartDashboard.putNumber("Velocity_X", navX.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", navX.getVelocityY());
    SmartDashboard.putNumber("Displacement_X", navX.getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", navX.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    SmartDashboard.putNumber("RawGyro_X", navX.getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", navX.getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", navX.getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", navX.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", navX.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", navX.getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", navX.getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", navX.getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", navX.getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", navX.getTempC());
    SmartDashboard.putNumber("IMU_Timestamp", navX.getLastSensorTimestamp());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    AHRS.BoardYawAxis yaw_axis = navX.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    SmartDashboard.putString("FirmwareVersion", navX.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    SmartDashboard.putNumber("QuaternionW", navX.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", navX.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", navX.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", navX.getQuaternionZ());

    /* Connectivity Debugging Support */
    SmartDashboard.putNumber("IMU_Byte_Count", navX.getByteCount());
    SmartDashboard.putNumber("IMU_Update_Count", navX.getUpdateCount());
    SmartDashboard.putNumber("IMU Rate", navX.getRate());
  }
}