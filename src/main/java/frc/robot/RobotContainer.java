package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import frc.robot.Constants.DRIVER;
import frc.robot.Constants.HARDWARE;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.Drive;

public class RobotContainer {

    // private final Joystick mLeftJoystick = new Joystick( HARDWARE.LEFT_JOYSTICK );;
    private final Joystick mRightJoystick = new Joystick( HARDWARE.RIGHT_JOYSTICK );;
    private final Drivetrain mDrivetrainSubsystem = new Drivetrain();
    private final SlewRateLimiter mSpeedLimiter = new SlewRateLimiter( DRIVER.DRIVE_SLEW_RATE_LIMITER );


    private double applyDeadband ( double value, double deadband ) {
        if ( Math.abs( value ) > deadband ) {
            if ( value > 0.0 ) {
                return (value - deadband) / ( 1.0 - deadband );
            } else {
                return ( value + deadband ) / ( 1.0 - deadband );
            }
        } else {
            return 0.0;
        }
    }
    
    private double GetDriveForward () {
        return mSpeedLimiter.calculate( applyDeadband( mRightJoystick.getX(), 
            DRIVER.JOYSTICK_DEADBAND ) ) * DRIVER.MAX_DRIVE_VELOCITY;
    }

    private double GetDriveStrafe () {
        return -mSpeedLimiter.calculate( applyDeadband( mRightJoystick.getY(),
            DRIVER.JOYSTICK_DEADBAND ) ) * DRIVER.MAX_DRIVE_VELOCITY;
    }

    private double GetDriveRotation () {
        return mSpeedLimiter.calculate( applyDeadband( mRightJoystick.getZ(),
            DRIVER.JOYSTICK_DEADBAND ) ) * DRIVER.MAX_ROTATION_VELOCITY;
    }

    public Drivetrain GetDrivetrainSubsystem () {
        return mDrivetrainSubsystem;
    }

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem( mDrivetrainSubsystem );
        CommandScheduler.getInstance().setDefaultCommand( mDrivetrainSubsystem, 
            new Drive( mDrivetrainSubsystem, GetDriveForward(), GetDriveStrafe(), GetDriveRotation(), true ) );
    }

}
