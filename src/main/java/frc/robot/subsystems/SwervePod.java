package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import frc.robot.utils.SparkMax;
import frc.robot.Constants.DRIVETRAIN;

public class SwervePod {
    
    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mTurnMotor;
    private final CANEncoder mDriveEncoder;
    private final boolean mIsDriveEncoderReversed;
    private final Encoder mTurnEncoder;
    private final DutyCycleEncoder mZeroingEncoder;
    private final PIDController mDrivePIDController;
    private final ProfiledPIDController mTurnPIDController;
    private NetworkTableEntry mDriveEncoderDistanceEntry;
    private NetworkTableEntry mDriveEncoderVelocityEntry;
    private NetworkTableEntry mTurnEncoderDistanceEntry;
    private NetworkTableEntry mTurnEncoderVelocityEntry;
    private NetworkTableEntry mZeroingEncoderDistanceEntry;


    /**
     * Output telemetry to the network tables.
     * */  
    public void OutputTelemetry () {
        mDriveEncoderDistanceEntry.setNumber( GetDrivePosition() );             // meters @ wheel
        mDriveEncoderVelocityEntry.setNumber( GetDriveVelocity() );             // meters per second @ wheel
        mTurnEncoderDistanceEntry.setNumber( mTurnEncoder.getDistance() );      // radians
        mTurnEncoderVelocityEntry.setNumber( mTurnEncoder.getRate() );          // radians per second
        mZeroingEncoderDistanceEntry.setNumber( mZeroingEncoder.getDistance()); // radians
    }


    /**
     * Returns the current state of the swerve pod.
     *
     * @return The current state of the serve pod.
     */
    public SwerveModuleState GetState () {
        return new SwerveModuleState( GetDriveVelocity(), new Rotation2d( mTurnEncoder.get() ) );
    }


    /**
     * Set the desired state of the swerve pod.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void SetDesiredState ( SwerveModuleState DesiredState ) {
        
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize( DesiredState, new Rotation2d( mTurnEncoder.get() ) );

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = mDrivePIDController.calculate(  GetDriveVelocity(), state.speedMetersPerSecond );

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = mTurnPIDController.calculate( mTurnEncoder.get(), state.angle.getRadians() );

        mDriveMotor.set( driveOutput );
        mTurnMotor.set( turnOutput );
    }


    /**
     * Zeros all the swerve pod encoders.
     */
    public void ResetEncoders() {
        mDriveEncoder.setPosition(0);
        mTurnEncoder.reset();
    }


    /**
     * Configure the PWM ducty cycle encoder and the quadaratutre encoder.
     */  
    private void ConfigureEncoders () {
        double baseConversion = Math.PI * Units.inchesToMeters( DRIVETRAIN.WHEEL_DIAMETER_INCH ) / DRIVETRAIN.GEAR_RATIO;
        mDriveEncoder.setPositionConversionFactor( baseConversion );            // rotations @ motor -> meters @ wheel
        mDriveEncoder.setVelocityConversionFactor( baseConversion / 60.0 );     // RPM @ motor -> meters per second @ wheel
        mTurnEncoder.setDistancePerPulse( 2.0 * Math.PI / 1024.0 );             // Degrees per encoder pulse
        mZeroingEncoder.setDistancePerRotation( 2 * Math.PI );
        // mTurnEncoder.setMinRate( 1.0 );                                         // Rate at which mechanism is considered "stopped"
        // mTurnEncoder.setSamplesToAverage( 5 );                                  // Angular velocity calculation
        // mTurnEncoder.setReverseDirection()
    }


    /**
     * 
     * @return
     */
    private double GetDriveVelocity () {
        return mIsDriveEncoderReversed ? -mDriveEncoder.getVelocity() : mDriveEncoder.getVelocity();
    }


    /**
     * 
     * @return
     */
    private double GetDrivePosition () {
        return mIsDriveEncoderReversed ? -mDriveEncoder.getPosition() : mDriveEncoder.getPosition();
    }    


    /**
     * The constructor for the SwervePod class.
     * @param swervePodID
     * @param driveMotorID
     * @param turnMotorID
     * @param quadAChannel
     * @param quadBChannel
     * @param pwmChannel
     * @param isDriveEncoderReversed
     */
    public SwervePod ( String swervePodID, int driveMotorID, int turnMotorID, int quadAChannel, int quadBChannel, int pwmChannel, boolean isDriveEncoderReversed ) {

        mIsDriveEncoderReversed = isDriveEncoderReversed;
        mDriveMotor = new CANSparkMax( driveMotorID, MotorType.kBrushless );
        mTurnMotor = new CANSparkMax( turnMotorID, MotorType.kBrushless );
        SparkMax.SetDefaultConfig( mDriveMotor );
        SparkMax.SetDefaultConfig( mTurnMotor );
        
        mDriveEncoder = mDriveMotor.getEncoder();
        mTurnEncoder  = new Encoder( quadAChannel, quadBChannel, false, CounterBase.EncodingType.k4X );
        mZeroingEncoder = new DutyCycleEncoder( pwmChannel );

        ConfigureEncoders();

        mDrivePIDController = new PIDController( DRIVETRAIN.DRIVE_P_GAIN, 0, DRIVETRAIN.DRIVE_D_GAIN );
        mTurnPIDController = new ProfiledPIDController( DRIVETRAIN.TURN_P_GAIN, 0, DRIVETRAIN.TURN_D_GAIN, 
            new TrapezoidProfile.Constraints( Units.degreesToRadians( DRIVETRAIN.MAX_TURN_VELOCITY_DPS ), 
                                              Units.degreesToRadians( DRIVETRAIN.MAX_TURN_ACCELERATION_DPSS ) ) );
        mTurnPIDController.enableContinuousInput( -Math.PI, Math.PI );

        mDriveEncoderDistanceEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Drive_Distance(m)");
        mDriveEncoderVelocityEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Drive_Velocity(mps)");
        mTurnEncoderDistanceEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Turn_Distance(rad)");
        mTurnEncoderVelocityEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Turn_Velocity(rad/s)");
        mZeroingEncoderDistanceEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Zeroing_Distance(rad)");
        
        mDriveMotor.set( 0.0 );
        mTurnMotor.set( 0.0 );

    }
    
}
