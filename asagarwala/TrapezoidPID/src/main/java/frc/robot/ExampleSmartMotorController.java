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


//Defines public class
public class ExampleSmartMotorController implements MotorController
{
  public enum PIDMode
  {
    //3 values
    kPosition, kVelocity, kMovementWitchcraft;
  }
  private static final double kEncoderCPR = 7200;
  private WPI_TalonSRX m_motor;

  /**
  private double kPosition = 0;
  private double kVelocity = 0;
  private double kMovementWitchcraft = 0;
  private int port; 
  */
 //activating motor 
  WPI_TalonSRX m_TopMotor = new WPI_TalonSRX(5);
  //Declares instance variables
  //PID
  //kp-original speed you start at
  private final static double kp = 0.05;
  private final static double ki =0.0;
  private final static double kd = 0.0;
  private double setpoint;
  private PIDMode mode;
  //private double arbFeedForward;


//defines enumerations(data type that contains predefined set of constants)
  

  /**
   * Creates a new ExampleSmartMotorController.
   * @param rotation
   * @param port The port for the controller.
   */

  public ExampleSmartMotorController(double kP, double kV, double kMV){
    
    

  }
//constructor takes integer port as parameter and assigns it to instance variable port
  @SuppressWarnings("PMD.UnusedFormalParameter")
  public ExampleSmartMotorController(int port) {
    m_TopMotor = new WPI_TalonSRX(port);

    m_TopMotor.config_kP(0, kp);
    m_TopMotor.config_kI(0, ki);
    m_TopMotor.config_kD(0, kd);
  }
  private double rotationsToCounts(double rotation){
    return rotation * kEncoderCPR;
  }

  private double countsToRotations(double encoderCounts){
    return encoderCounts / kEncoderCPR;
  }

  



  /**
   * Example method for setting the PID gains of the smart controller.
   *
   * @param kp The proportional gain.
   * @param ki The integral gain.
   * @param kd The derivative gain.
   */
  
//sets PID gain parameters
/** 
public void setPID(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;

  }
*/
  /**
   * Example method for setting the setpoint of the smart controller in PID mode.
   *
   * @param mode The mode of the PID controller.
   * @param setpoint The controller setpoint.
   * @param arbFeedforward An arbitrary feedforward output (from -1 to 1).
   */
  //sets Setpoint
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
  }

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
  //public double getEncoderDistance() {
    //return arbFeedForward;

  //}

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public void getEncoderRate() {

  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder() {
  }

  public void set(double speed) {}

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
  public void setPID(double d, double e, double f) {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'setPID'");
  }
}
