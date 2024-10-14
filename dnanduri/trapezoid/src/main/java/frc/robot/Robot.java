// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  //setting Xbox

  //importing the TalonSRX motors
  private final MotorController a_leftRearMotor = new WPI_TalonSRX(1);
  private final MotorController a_leftFrontMotor = new WPI_TalonSRX(2);
  private final MotorController a_rightRearMotor = new WPI_TalonSRX(3);
  private final MotorController a_rightFrontMotor = new WPI_TalonSRX(4);

  MotorControllerGroup a_leftMotorGroup = new MotorControllerGroup(a_leftFrontMotor, a_leftRearMotor);
  MotorControllerGroup a_rightMotorGroup = new MotorControllerGroup(a_rightFrontMotor, a_rightRearMotor);

  SlewRateLimiter xFilter = new SlewRateLimiter(1);
  SlewRateLimiter yFilter = new SlewRateLimiter(1);

  private static double kDt = 0.02;

  //create controller
  private final XboxController controller = new XboxController(0);
  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(a_leftMotorGroup::set, a_rightMotorGroup::set);

  @Override
  //This activates whenever the code is deployed to the robot
  public void robotInit() {
    // Note: These gains are fake, and will have to be tuned for your robot.
    
    //starts rumble when code is deployed
    controller.setRumble(RumbleType.kLeftRumble, 1.0);
    controller.setRumble(RumbleType.kRightRumble, 1.0);

    a_rightMotorGroup.setInverted(true);
    WPI_TalonSRX leftRearMotor = (WPI_TalonSRX) a_leftRearMotor;
    leftRearMotor.config_kP(0, 1.3); // set proportional gain
    leftRearMotor.config_kI(0, 0.0); // set integral gain
    leftRearMotor.config_kD(0, 0.7); // set derivative gain}
  

    }

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, a_leftMotorGroup);
    SendableRegistry.addChild(m_robotDrive, a_rightMotorGroup);
  }

  @Override
  public void teleopPeriodic() {
    if (controller.getRawButtonPressed(2)) {
      m_goal = new TrapezoidProfile.State(5, 0);
    } else if (controller.getRawButtonPressed(3)) {
      m_goal = new TrapezoidProfile.State();
    }
    SmartDashboard.putNumber("Left Motor Group", a_leftMotorGroup.get());
    SmartDashboard.putNumber("Right Motor Group", a_rightMotorGroup.get());
    SmartDashboard.putNumber("Slew Y", yFilter.lastValue());
    SmartDashboard.putNumber("Slew X", xFilter.lastValue());


    //This code processes joystick inputs for a robot's arcade drive system. 
    //It applies filters to smooth out the inputs, uses a deadband to ignore small unintended movements, and scales the inputs down for finer control. 
    //Finally, it clamps the values to ensure they stay within the range of -1.0 to 1.0, allowing for controlled and responsive driving.
    yFilter.calculate(controller.getLeftY());
    xFilter.calculate(controller.getRightX());
    m_robotDrive.arcadeDrive(yFilter.calculate(controller.getLeftY()), controller.getRightX());

    m_robotDrive.arcadeDrive(MathUtil.applyDeadband(controller.getLeftY(), 0.1) * 0.5, MathUtil.applyDeadband(controller.getRightX(), 0.1) * 0.5, false);
    
    double leftY = MathUtil.clamp(MathUtil.applyDeadband(controller.getLeftY(), 0.1), -1.0, 1.0);
    double rightX = MathUtil.clamp(MathUtil.applyDeadband(controller.getRightX(), 0.1), -1.0, 1.0);

    leftY = yFilter.calculate(Math.abs(leftY) * (leftY) * 0.5);
    rightX = xFilter.calculate(Math.abs(rightX) * (rightX) * 0.5);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    // Send setpoint to offboard controller PID
    WPI_TalonSRX leftRearMotor = (WPI_TalonSRX) a_leftRearMotor;
    leftRearMotor.set(ControlMode.Position, m_setpoint.position, 
       DemandType.ArbitraryFeedForward, m_feedforward.calculate(m_setpoint.velocity) / 12.0);


}
}