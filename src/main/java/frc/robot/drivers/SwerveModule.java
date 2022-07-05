package frc.robot.drivers;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import frc.robot.utils.SparkMax;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.CONTROL;

public class SwerveModule {

    private final double mHome;
    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mTurnMotor;
    private final CANEncoder mDriveEncoder;
    private final boolean mIsDriveEncoderReversed;
    private final boolean mIsTurnEncoderReversed;
    private final Encoder mTurnEncoder;
    private final DutyCycle mZeroingEncoder;
    private final PIDController mDrivePIDController;
    private final ProfiledPIDController mTurnPIDController;
    private double mTurnTarget;
    private double mDriveTarget;
    private double mTurnOutput;
    private double mDriveOutput;


    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Get the absolute encoder position of the turning encoder. The value will
     * be between [0, 2*pi].
     * @return the absolute encoder position (radians)
     */
    public double GetTurnAbsolutePosition () {
        return mZeroingEncoder.getOutput() * 2.0 * Math.PI;
    }  

    /**
     * Get the turning encoder distance since the last reset. A small value is
     * added to the encoder value to such that a value will never land on 0.0,
     * PI/2, PI, 3*PI/2, 2*PI, ... During PID tuning, these generated unwanted
     * oscillations.
     * @return the turning encoder distance (radians)
     */
    public double GetTurnDistance () {
        return mIsTurnEncoderReversed ? -( mTurnEncoder.getDistance() + 0.00001 )
                                      : mTurnEncoder.getDistance() + 0.00001;
    }

    /**
     * Get the drive encoder velocity.
     * @return The drive encoder velocity (meters/second)
     */
    public double GetDriveVelocity () {
        return mIsDriveEncoderReversed ? -mDriveEncoder.getVelocity()
                                       : mDriveEncoder.getVelocity();
    }

    /**
     * Check if the turning controller is at the goal.
     * @return true if at the goal, false otherwise
     */
    public synchronized boolean GetTurnControllerAtGoal () {
        return mTurnPIDController.atGoal();
    }  

    /**
     * Update the turning target setpoint.
     * @param turnTarget the new turning target setpoint (radians)
     */
    public synchronized void SetTurnTarget ( double turnTarget ) {
        mTurnTarget = turnTarget;
    }

    /**
     * Get the turning target setpoint.
     * @return the turning target setpoint (radians)
     */
    public synchronized double GetTurnTarget () {
        return mTurnTarget;
    }

    /**
     * Update the driving target setpoint
     * @param driveTarget the new driving target (meters per second)
     */
    public synchronized void SetDriveTarget ( double driveTarget ) {
        mDriveTarget = driveTarget;
    }


    public synchronized void SetTurnOutput ( double turnOutput ) {
        mTurnOutput = turnOutput;
    }
    public synchronized double GetTurnOutput () {
        return mTurnOutput;
    }

    public synchronized void SetDriveOutput ( double driveOutput ) {
        mDriveOutput = driveOutput;
    }
    public synchronized double GetDriveOutput () {
        return mDriveOutput;
    }


    /**
     * Returns the current state of the swerve module.
     *
     * @return the current state of the serve module (velocity: meters per
     * second, heading: radians)
     */
    public SwerveModuleState GetState () {
        return new SwerveModuleState( GetDriveVelocity(), 
                                      new Rotation2d( GetTurnDistance() ) );
    }

    /**
     * Get the driving target setpoint.
     * @return the driving target setpoint (meters per second)
     */
    public synchronized double GetDriveTarget () {
        return mDriveTarget;
    }

    /**
     * Get the calibrated home position of the swerve module in radians between
     * [0, 2*pi]
     * @return the home position (radians)
     */
    public double GetHomePosition () {
        return mHome;
    }

    /**
     * Set the drive encoder position to 0 and reset the turning encoder
     * counter to 0.
     */
    public void ResetEncoders () {
        mDriveEncoder.setPosition(0);
        mTurnEncoder.reset();
    }

    /**
     * Set the driving and tuning motor output to 0.
     */
    public void SetNullOutput () {
        mDriveMotor.set( 0.0 );
        mTurnMotor.set( 0.0 );
    }

    /**
     * Set the desired state of the swerve module (drive in meters per second
     * and heading in radians).
     * @param desiredState the desired state of the swerve module
     */
    public void SetState ( SwerveModuleState DesiredState ) {

        SwerveModuleState state = SwerveModuleState.optimize( DesiredState,
            new Rotation2d( GetTurnDistance() ) );

        SetDriveTarget( state.speedMetersPerSecond );            
        SetTurnTarget( state.angle.getRadians() );

        double driveOutput = mDrivePIDController.calculate(  GetDriveVelocity(),
            state.speedMetersPerSecond );
        
        double turnOutput = mTurnPIDController.calculate( GetTurnDistance(),
            state.angle.getRadians() );

        SetDriveOutput( driveOutput );
        SetTurnOutput( turnOutput );

        mDriveMotor.set( mIsDriveEncoderReversed ? -driveOutput : driveOutput );
        mTurnMotor.set( turnOutput );
    }


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Configure the encoders. The PWM duty cycle encoder and the quadrature
     * encoder are wired from teh CTRE mag encoder to the DIO's on the RoboRio.
     * The drive encoder uses the integrated motor encoder of the REV NEO
     * motor.
     */  
    private void ConfigureEncoders () {
        double baseConversion = 
            Math.PI * 
            Units.inchesToMeters( DRIVETRAIN.WHEEL_DIAMETER_INCH ) / 
            DRIVETRAIN.DRIVE_GEAR_RATIO;
        mDriveEncoder.setPositionConversionFactor( baseConversion );            // rotations @ motor -> meters @ wheel
        mDriveEncoder.setVelocityConversionFactor( baseConversion / 60.0 );     // RPM @ motor -> meters per second @ wheel
        mTurnEncoder.setDistancePerPulse( 2.0 * Math.PI / 1024.0 );             // Degrees per encoder pulse
    }


    //-------------------------------------------------------------------------------------------//
    /*                                      CONSTRUCTOR                                          */
    //-------------------------------------------------------------------------------------------//

    
    /**
     * The constructor for the Drivetrain class.
     * @param driveMotorID the drive motor CAN ID
     * @param turnMotorID the turning motor CAN ID
     * @param quadAChannel the DIO channel of the encoder quadrature A signal
     * @param quadBChannel the DIO channel of the encoder quadrature B signal
     * @param pwmChannel the DIO channel of the encoder PWM signal
     * @param isDriveEncoderReversed a flag to negate the drive encoder values
     * @param isTurnEncoderReversed a flag to negate the turning encoder values
     */
    public SwerveModule ( ShuffleboardLayout layout, int driveMotorID, int turnMotorID,
                          double home, int quadAChannel, int quadBChannel, int pwmChannel,
                          boolean isDriveEncoderReversed, boolean isTurnEncoderReversed ) {

        mDriveMotor = new CANSparkMax( driveMotorID, MotorType.kBrushless );
        mTurnMotor = new CANSparkMax( turnMotorID, MotorType.kBrushless );
        mDriveEncoder = mDriveMotor.getEncoder();
        mTurnEncoder  = new Encoder( quadAChannel, quadBChannel, false, CounterBase.EncodingType.k4X );
        mZeroingEncoder = new DutyCycle( new DigitalInput( pwmChannel ) );
        mDrivePIDController = new PIDController( DRIVETRAIN.DRIVE_P_GAIN, 0, DRIVETRAIN.DRIVE_D_GAIN, CONTROL.LOOP_TIME_S );
        mTurnPIDController = new ProfiledPIDController( DRIVETRAIN.TURN_P_GAIN, 0, DRIVETRAIN.TURN_D_GAIN, 
            new TrapezoidProfile.Constraints( DRIVETRAIN.MAX_TURN_VELOCITY_RPS, 
                                              DRIVETRAIN.MAX_TURN_ACCELERATION_RPSS ), CONTROL.LOOP_TIME_S );

        SparkMax.SetDefaultConfig( mDriveMotor );
        SparkMax.SetDefaultConfig( mTurnMotor );
        mTurnPIDController.enableContinuousInput( -Math.PI, Math.PI );
        mTurnPIDController.setTolerance( Units.degreesToRadians( DRIVETRAIN.MAX_TURN_ERROR_DEG ) );
        ConfigureEncoders();
        ResetEncoders();

        mHome = home;
        mIsDriveEncoderReversed = isDriveEncoderReversed;
        mIsTurnEncoderReversed = isTurnEncoderReversed;

        layout.addNumber( "TurnAbsolute", () -> GetTurnAbsolutePosition() );
        layout.addNumber( "TurnQuad", () -> GetTurnDistance() );
        layout.addNumber( "TurnTarget", () -> GetTurnTarget() );
        layout.addBoolean( "TurnAtGoal", () -> GetTurnControllerAtGoal() );
        layout.addNumber( "TurnHome", () -> GetHomePosition() );
        layout.addNumber( "DriveTarget", () -> GetDriveTarget() );
        layout.addNumber( "DriveOutput", () -> GetDriveOutput() );
        layout.addNumber( "TurnOutput", () -> GetTurnOutput() );
        layout.addNumber( "DriveVelocity", () -> GetDriveVelocity() );                            
        
        

    }

}
