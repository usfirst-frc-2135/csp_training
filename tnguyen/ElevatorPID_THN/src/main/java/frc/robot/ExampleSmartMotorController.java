// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * A simplified stub class that simulates the API of a common "smart" motor controller.
 *
 * <p>
 * Has no actual functionality.
 */
public class ExampleSmartMotorController
{
  // TODO: Keep your constants and variables at the top of the class--it's where people look for them
  //      I moved PIDMode and m_motorSim up here

  /**
   * declares PIDMode as an enum
   * PIDMode used to determine controlmode
   */
  public enum PIDMode
  {
    kPosition, kVelocity, kMovementWitchcraft
  }

  // TODO:  Since we don't want to have this configure a specific motor controller (5) as a default
  //        the "port" definition coming in will initialize a WPI_TalonSRX with the correct port
  //        also, "final" means this m_motor cannot be changed (ever), so just declare an m_motor
  //        variable without "final" and without the initialization "= new WPI_TalonSRX(5)"
  private final WPI_TalonSRX    m_motor = new WPI_TalonSRX(5);
  private TalonSRXSimCollection m_motorSim;

  private double                m_kp;
  private double                m_ki;
  private double                m_kd;

  private static double         m_kEncoderCPR;  // TODO: Since this is no longer a constant (within this class), remove the "k"
  // And the "static" is probably not needed (although won't cause an error)

  // TODO: Keep your constructors at the top of the class--it's where everyone will look for them
  //      I moved yours here

  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port
   *          The port for the controller.
   */
  @SuppressWarnings("PMD.UnusedFormalParameter")
  public ExampleSmartMotorController(int ports, double kEncoderCPR)
  {
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    //encoder type in TalonSRX is quadrature encoder
    m_motor.selectProfileSlot(0, 0);

    setPID(m_kp, m_ki, m_kd);

    m_motorSim = m_motor.getSimCollection( );
    m_kEncoderCPR = kEncoderCPR;
  }

  // TODO: Our new usage now requires a port and encoder parameter
  //        delete this now unused constructor interface below
  public ExampleSmartMotorController( )
  {};

  /**
   * COUNTS TO ROTATIONS
   * encoder reads values in counts, but human users input rotations for simplicity
   * 4096 counts in one rotation, following methods converts counts to rotations, vice versa
   */

  private double rotationsToCounts(double rotations)
  {
    return rotations * m_kEncoderCPR;
  }

  private double countsToRotations(double encoderCounts)
  {
    return encoderCounts / m_kEncoderCPR;
  }

  /**
   * Example method for setting the PID gains of the smart controller.
   *
   * @param kp
   *          The proportional gain.
   * @param ki
   *          The integral gain.
   * @param kd
   *          The derivative gain.
   */
  public void setPID(double kp, double ki, double kd)
  {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_motor.config_kP(0, kp);
    m_motor.config_kI(0, ki);
    m_motor.config_kD(0, kd);
    // TODO: It would be nice to log when the PID is changed, yes?
  }

  public double getKp( )
  {
    return m_kp;
  }

  /**
   * Example method for setting the setpoint of the smart controller in PID mode.
   *
   * @param mode
   *          The mode of the PID controller.
   * @param setpoint
   *          The controller setpoint.
   * @param arbFeedforward
   *          An arbitrary feedforward output (from -1 to 1).
   */
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward)
  {
    ControlMode controlMode;

    switch (mode)
    {
      default :
      case kPosition :
        controlMode = ControlMode.Position;
        break;

      case kVelocity :
        controlMode = ControlMode.Velocity;
        setpoint /= 10;
        break;

      case kMovementWitchcraft : // is not used in this code
        controlMode = ControlMode.MotionMagic;
        break;
    }

    m_motor.set(controlMode, rotationsToCounts(setpoint));
    // TODO: You'll find that when you log stuff on separate lines, it increases the "spam" factor to the console
    //        Put this on one line. This would also make it easy to cut and paste in to a spreadsheet for plotting if needed.
    //        And just display the passed in setpoint, not the value going to the motors
    DataLogManager.log("ControlMode: " + controlMode + " Setpoint: " + setpoint);
    // DataLogManager.log("ControlMode: " + controlMode);
    // DataLogManager.log("Setpoint: " + rotationsToCounts(setpoint));
  }

  /**
   * Places this motor controller in follower mode.
   *
   * @param leader
   *          The leader to follow.
   */
  public void follow(ExampleSmartMotorController leader)
  {}

  /**
   * Returns the encoder distance.
   *
   * @return The current encoder distance.
   */
  public double getEncoderDistance( )
  {
    return countsToRotations(m_motor.getSelectedSensorPosition(0));
  }

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public double getEncoderRate( )
  {
    return countsToRotations(m_motor.getSelectedSensorVelocity(0) * 10);
  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder( )
  {
    // TODO: It would be nice to log when the encoder is reset, yes?
    m_motor.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * used to set the constant speed of the motor, in percentOutput
   */
  public void set(double voltage)
  { // set the speed of the motor using percent output
   // TODO: It would be nice to log when the motor speed is changed, yes?
    m_motor.set(ControlMode.PercentOutput, voltage);
  }

  public double get( )
  {
    return m_motor.getMotorOutputPercent( );
  }

  /** Inverts motor direction */
  public void setInverted(boolean isInverted)
  {
    // TODO: It would be nice to log when the inversion is changed, yes?
    m_motor.setInverted(isInverted);
  }

  public boolean getInverted( )
  {
    return m_motor.getInverted( );
  }

  public double getClosedLoopError( )
  {
    return (m_motor.getClosedLoopError( ));
  }

  public void disable( )
  {
    // TODO: It would be nice to log when the motor is disabled, yes?
    m_motor.set(ControlMode.Disabled, 0);
  }

  public double getVelocity( )
  {
    return m_motor.getSelectedSensorVelocity( );
  }

  public void stopMotor( )
  {
    // TODO: It would be nice to log when the motor is stopped, yes?
    set(0.0);
  }

  /**
   * Return the motor simulation object for this controller
   *
   * @return motor simulation object
   */
  public TalonSRXSimCollection getMotorSimulation( )
  {
    return m_motorSim;
  }

}
