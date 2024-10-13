//This whole section of code is mean to spin the motor on top of the test bot using PID

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenix.motorcontrol.ControlMode;


/**
 * A simplified stub class that simulates the API of a common "smart" motor controller.
 *
 * <p>Has no actual functionality.
 */
public class ExampleSmartMotorController {
  WPI_TalonSRX a_extraMotor = new WPI_TalonSRX(5);
  private double kp = 0.05;
  private double ki = 0;
  private double kd = 0;

  //enum is short for enumeration, which means the action of mentioning a number of things one by one
  //Often represents specific categories or states, which is why the variables kPosition, kVelocity, and kMovementWitchcraft were placed here
  //They prevent errors from arbitrary strings or integers
  //enhances code readability, which makes it easier to read
  //Can be used in switch statements
  public enum PIDMode {
    kPosition,
    kVelocity,
    kMovementWitchcraft,
  }
  private static final double kEncoderCPR = 360;
  private WPI_TalonSRX m_motor;
  private double setpoint;
  private PIDMode mode;


  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port The port for the controller.
   */
  @SuppressWarnings("PMD.UnusedFormalParameter")
  //The constructor initialises the motor with the given PID settings
  //It ensures the motor is ready to operate with these configurations
  //The unused port parameter may be intended for future use
  public ExampleSmartMotorController(int port) { //This is the constructor; it specifies the CAN bus port that the motor controller is connected to


    a_extraMotor = new WPI_TalonSRX(port);
    a_extraMotor.config_kP(0, kp);
    a_extraMotor.config_kI(0, ki);
    a_extraMotor.config_kD(0, kd);
    //These lines configure the PID gains for the motor controller
    //kp, ki, and kd are the proportional, integral, and derivative gains that correspond to how the motor responds to error in the control loop
  }

  /**
   * Example method for setting the PID gains of the smart controller.
   *
   * @param kp The proportional gain.
   * @param ki The integral gain.
   * @param kd The derivative gain.
   */

   //Allows for dynamic adjustment of the PID gains after the object has been created
   //Users of this class can customise the controller's behaviour without having to create a new instance of ExampleSmartMotorController
  public void setPID(double kp, double ki, double kd) {
    this.kp = kp; //kp is the initial speed
    this.ki = ki; //ki is the initial speed increase
    this.kd = kd; //kd is the speed decrease at the end
  }

  private double rotationsToCounts(double rotation) {
    return rotation * kEncoderCPR;
  }

  private double countsToRotation(double encoderCounts) {
    return encoderCounts / kEncoderCPR;
  }

  /**
   * Example method for setting the setpoint of the smart controller in PID mode.
   *
   * @param mode The mode of the PID controller.
   * @param setpoint The controller setpoint.
   * @param arbFeedforward An arbitrary feedforward output (from -1 to 1).
   */

   //setSetpoint is used to manage motor performance
   //PIDMode is an enum that determines which control mode to use (position/velocity/etc.)
   //double setpoint is the target value the system should try to achieve
   //double arbFeedForward is a value that could be used to add feedforward control, which helps the system be more efficient
   //because it anticipates the required control effort
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward) {
    ControlMode controlMode;

    //uses a switch statement to determine the appropriate control mode based on the provided mode
    //If the mode is unspecified, it defaults to kPosition
    //kPosition sets the control mode to the position control, where the system aims to reach a specified position
    switch (mode) {
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
    a_extraMotor.set(controlMode, rotationsToCounts(setpoint));
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
  public void getEncoderDistance() {}

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public void getEncoderRate() {}

  /** Resets the encoder to zero distance. */
  public void resetEncoder() {}

  public void set(double speed) {}

  public double get() {
    return 0;
  }

  public void setInverted(boolean isInverted) {
    a_extraMotor.setInverted(isInverted);
  }

  public boolean getInverted() {
    return a_extraMotor.getInverted( );
  }

  public void disable() {
    a_extraMotor.set(ControlMode.Disabled, 0);
  }

  public void stopMotor() {
    set(0.0);
  }
}
