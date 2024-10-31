// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  //set time increment for control loop (2 milliseconds)
  private static double kDt = 0.02; 

  //define four motors as WPI_TalonSRX, create xbox controller, 
  private final MotorController m_leftRearMotor = new WPI_TalonSRX(1);
  private final MotorController m_leftFrontMotor = new WPI_TalonSRX(2);
  private final MotorController m_rightRearMotor = new WPI_TalonSRX(3);
  private final MotorController m_rightFrontMotor = new WPI_TalonSRX(4);
  private XboxController controller; 
  
  //create instance of SimpleMotorFeedForward -> calculate feedforward control of motor
  //anticipate the necessary motor power to achieve the desired motion -> smoother + more responsive control
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5); 
  //ks (static gain) = starting power to overcome friction : 1 volt
  //kv (velocity gain) = how much extra voltage for each unit of speed you want the motor to achieve : 1.5 volts per unit of speed
  
  //create left + right motor groups
  MotorControllerGroup m_leftMotorGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
  MotorControllerGroup m_rightMotorGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);
  
  //set slew rate limiter- limits the rate of change for joystick inputs
  SlewRateLimiter xFilter = new SlewRateLimiter(2);
  SlewRateLimiter yFilter = new SlewRateLimiter(2);

  private final DifferentialDrive m_robotDrive = 
  new DifferentialDrive(m_leftMotorGroup::set, m_rightMotorGroup::set);

  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(); //goal: desired state (position, velocity)
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(); //setpoint: current position adn velocity as you move towards that goal

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotorGroup);
    SendableRegistry.addChild(m_robotDrive, m_rightMotorGroup);
  }

  @Override
  public void robotInit() {
    m_rightMotorGroup.setInverted(true);
    WPI_TalonSRX leftRearMotor = (WPI_TalonSRX) m_leftRearMotor;
    leftRearMotor.config_kP(0, 1.3); // set proportional gain
    leftRearMotor.config_kI(0, 0.0); // set integral gain
    leftRearMotor.config_kD(0, 0.7); // set derivative gain
  }

  @Override
  public void teleopPeriodic() {
    if (controller.getYButtonPressed()) {
      m_goal = new TrapezoidProfile.State(5, 0);
    } else if (controller.getXButtonPressed()) { 
      m_goal = new TrapezoidProfile.State();
    }

    //apply deadband (0.1) and clamp left y and right x between (-1, 1)
    double leftY = MathUtil.clamp(MathUtil.applyDeadband(controller.getLeftY(), 0.1), -1.0, 1.0);
    double rightX = MathUtil.clamp(MathUtil.applyDeadband(controller.getRightX(), 0.1), -1.0, 1.0);

    //add slew rate limiter to x and y and scale motor speed by half
    leftY = yFilter.calculate(Math.abs(leftY) * (leftY) * 0.5);
    rightX = xFilter.calculate(Math.abs(rightX) * (rightX) * 0.5);

    //drive + turn off square inputs
    m_robotDrive.arcadeDrive(leftY, rightX, false);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    // Send setpoint to offboard controller PID
    WPI_TalonSRX leftRearMotor = (WPI_TalonSRX) m_leftRearMotor;
      leftRearMotor.set(ControlMode.Position, m_setpoint.position, 
         DemandType.ArbitraryFeedForward, m_feedforward.calculate(m_setpoint.velocity) / 12.0);
    
    //add widgets
    SmartDashboard.putNumber("Right Motor Group", m_rightMotorGroup.get());
    SmartDashboard.putNumber("Left Motor Group", m_leftMotorGroup.get());
    SmartDashboard.putNumber("X slew rate", xFilter.lastValue());
    SmartDashboard.putNumber("Y slew rate", yFilter.lastValue());
  }
}
