// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
/**
 * A simplified stub class that simulates the API of a common "smart" motor controller.
 *
 * <p>Has no actual functionality.
 */


public class ExampleSmartMotorController implements MotorController{
  public enum PIDMode {
    kPosition,
    kVelocity,
    kMovementWitchcraft
  }
  private static final double kEncoderCPR = 4096;
  private WPI_TalonSRX m_motor;
  WPI_TalonSRX m_TopMotor = new WPI_TalonSRX(5);
  private static double kp = 0.05;
  private static double ki = 0;
  private static double kd = 0;
  private double setpoint;
  private PIDMode mode;

  public ExampleSmartMotorController(double kP, double kV, double kMV){

  }
 

  public ExampleSmartMotorController(int port) {
    m_TopMotor = new WPI_TalonSRX(port);
    //TODO Auto-generated constructor stub
    m_TopMotor.config_kP(0, kp);
    m_TopMotor.config_kI(0, ki);
    m_TopMotor.config_kD(0, kd);
}

/**
   * Creates a new ExampleSmartMotorController.
   * @param rotation
   * @param port The port for the controller.
   */
  
  private double rotationsToCounts(double rotations) {
    return rotations * kEncoderCPR;

  }

  private double countsToRotations(double encoderCounts) {
    return encoderCounts / kEncoderCPR;

  }

  /* 
  private double kp;
  private double ki;
  private double kd;
*/
/*
  public void ExampleSmartMotorController(int port) {
    m_TopMotor = new WPI_TalonSRX(port);
    //TODO Auto-generated constructor stub
  }
 */
  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port The port for the controller.
   */
  @SuppressWarnings("PMD.UnusedFormalParameter")

  
  //public ExampleSmartMotorController(int port) {}

  /**
   * Example method for setting the PID gains of the smart controller.
   *
   * @param kp The proportional gain.
   * @param ki The integral gain.
   * @param kd The derivative gain.
   */
  public void setPID(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }
  

  /**
   * Example method for setting the setpoint of the smart controller in PID mode.
   *
   * @param mode The mode of the PID controller.
   * @param setpoint The controller setpoint.
   * @param arbFeedforward An arbitrary feedforward output (from -1 to 1).
   */
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward) {
    ControlMode controlMode;
    switch(mode){
      default :
      case kPosition :
        controlMode = ControlMode.Position;
        break;

      case kVelocity :
        controlMode = ControlMode.Velocity;
        setpoint /= 10;
        break;

      case kMovementWitchcraft :
        controlMode = ControlMode.MotionMagic;
        break;
    }
    m_TopMotor.set(controlMode, rotationsToCounts(setpoint));
  } // this one will require you to define PIDMode as an enum
   


  /**
   * Places this motor controller in follower mode.
   *
   * @param leader The leader to follow.
   */
  public void follow(ExampleSmartMotorController leader) {

  }

  /**
   * Returns the encoder distance.
   *
   * @return The current encoder distance.
   */
  public double getEncoderDistance() {
    return rotationsToCounts(1);
  }

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public double getEncoderRate() {
    return countsToRotations(4096);
  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder() {}

  public void set(double voltage) {
    m_TopMotor.set(ControlMode.PercentOutput, voltage);
  }

  public double get() {
    return 0;
  }

  public void setInverted(boolean isInverted) {
    m_TopMotor.setInverted(isInverted);
  }

  public boolean getInverted() {
    return m_TopMotor.getInverted();
  }

  public void disable() {
    m_TopMotor.set(ControlMode.Disabled, 0);
  }

  public void stopMotor() {
    set(0.0);
  }
}
