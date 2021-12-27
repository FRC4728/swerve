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

    private final String id;
    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mTurnMotor;
    private final CANEncoder mDriveEncoder;
    private final boolean mIsDriveEncoderReversed;
    private final boolean mIsTurnEncoderReversed;
    private final Encoder mTurnEncoder;
    private final DutyCycleEncoder mZeroingEncoder;
    private final PIDController mDrivePIDController;
    private final ProfiledPIDController mTurnPIDController;
    private double mHomingGoal;
    private boolean mIsHomingFinshed;
    private double mDriveOutput;
    private double mTurnOutput;
    private NetworkTableEntry mDriveEncoderDistanceEntry;
    private NetworkTableEntry mDriveEncoderVelocityEntry;
    private NetworkTableEntry mTurnEncoderDistanceEntry;
    private NetworkTableEntry mTurnEncoderVelocityEntry;
    private NetworkTableEntry mZeroingEncoderDistanceEntry;
    private NetworkTableEntry mHomingEntry;
    private NetworkTableEntry mHomingGoalEntry;
    private NetworkTableEntry mDriveOutputEntry;
    private NetworkTableEntry mTurnOutputEntry;


    /**
     * Output telemetry to the network tables.
     * */  
    public void OutputTelemetry () {
        mDriveEncoderDistanceEntry.setNumber( GetDriveDistance() );             // meters @ wheel
        mDriveEncoderVelocityEntry.setNumber( GetDriveVelocity() );             // meters per second @ wheel
        mTurnEncoderDistanceEntry.setNumber( GetTurnDistance() );               // radians
        mTurnEncoderVelocityEntry.setNumber( GetTurnVelocity() );               // radians per second
        mZeroingEncoderDistanceEntry.setNumber( mZeroingEncoder.getDistance()); // radians
        mHomingEntry.setBoolean( mIsHomingFinshed );
        mHomingGoalEntry.setNumber( mHomingGoal );
        mDriveOutputEntry.setNumber( mDriveOutput );
        mTurnOutputEntry.setNumber( mTurnOutput );
    
    }


    /**
     * Returns the current state of the swerve pod.
     *
     * @return The current state of the serve pod.
     */
    public SwerveModuleState GetState () {
        return new SwerveModuleState( GetDriveVelocity(), new Rotation2d( GetTurnDistance() ) );
    }


    /**
     * Set the desired state of the swerve pod.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void SetDesiredState ( SwerveModuleState DesiredState ) {
        
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize( DesiredState, new Rotation2d( GetTurnDistance() ) );

        //SwerveModuleState state = DesiredState;

        // Calculate the drive output from the drive PID controller.
        mDriveOutput = mDrivePIDController.calculate(  GetDriveVelocity(), state.speedMetersPerSecond );

        // Calculate the turning motor output from the turning PID controller.
        mTurnOutput = mTurnPIDController.calculate( GetTurnDistance(), state.angle.getRadians() );

        mDriveMotor.set( mDriveOutput );
        mTurnMotor.set( mTurnOutput );
        

        // if ( id.equals("FrontLeft")) {
        //     // mDriveMotor.set( 0.0 );
        //     // mTurnMotor.set( mTurnOutput );
        //     System.out.println( DesiredState.toString() );
        //     //System.out.println( "Output: "+mTurnOutput);
        // }
    }
    
    
    
    public void StartHoming () {
        mTurnPIDController.setTolerance( Units.degreesToRadians( 3.0 ) );
        mTurnPIDController.reset( GetTurnDistance() );
        mHomingGoal = mHomingGoal - mZeroingEncoder.getDistance();
        mTurnPIDController.setGoal( new TrapezoidProfile.State( mHomingGoal, 0.0) );
        HomingUpdate();
    }

    public void HomingUpdate () {
        if ( mTurnPIDController.atGoal() ) {
            mIsHomingFinshed = true;
            mDriveOutput = 0.0;
            mTurnOutput = 0.0;
            mDriveMotor.set( mDriveOutput );
            mTurnMotor.set( mTurnOutput );
            ResetEncoders();          
        } 
        else {
            SetDesiredState( new SwerveModuleState( 0.0, new Rotation2d( mHomingGoal ) ) );
        }
    }

    public boolean IsHomingFinshed () {
        return mIsHomingFinshed;
    }

    
    // public void ResetHoming () {
    //     DriverStation.reportWarning( "Resetting swerve pod homing", false);
    // }



    /**
     * 
     * @return
     */
    private double GetTurnVelocity () {
        return mIsTurnEncoderReversed ? -mTurnEncoder.getRate() : mTurnEncoder.getRate();
    }

    /**
     * 
     * @return
     */    
    private double GetTurnDistance () {
        return mIsTurnEncoderReversed ? -mTurnEncoder.getDistance() + 0.00001 : mTurnEncoder.getDistance() + 0.00001;
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
    private double GetDriveDistance () {
        return mIsDriveEncoderReversed ? -mDriveEncoder.getPosition() : mDriveEncoder.getPosition();
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
     * The constructor for the SwervePod class.
     * @param swervePodID
     * @param driveMotorID
     * @param turnMotorID
     * @param quadAChannel
     * @param quadBChannel
     * @param pwmChannel
     * @param isDriveEncoderReversed
     */
    public SwervePod ( String swervePodID, int driveMotorID, int turnMotorID, int quadAChannel, int quadBChannel, int pwmChannel, boolean isDriveEncoderReversed, boolean isTurnEncoderReversed, double zero_rad ) {
        id = swervePodID;
        mIsHomingFinshed = false;
        mHomingGoal = zero_rad;
        mIsDriveEncoderReversed = isDriveEncoderReversed;
        mIsTurnEncoderReversed = isTurnEncoderReversed;
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
            new TrapezoidProfile.Constraints( DRIVETRAIN.MAX_TURN_VELOCITY_RPS, 
                                              DRIVETRAIN.MAX_TURN_ACCELERATION_RPSS ) );
        mTurnPIDController.enableContinuousInput( -Math.PI, Math.PI );

        mDriveEncoderDistanceEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Drive_Distance(m)");
        mDriveEncoderVelocityEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Drive_Velocity(mps)");
        mTurnEncoderDistanceEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Turn_Distance(rad)");
        mTurnEncoderVelocityEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Turn_Velocity(rad/s)");
        mZeroingEncoderDistanceEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Zeroing_Distance(rad)");
        mHomingEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Homing_Finished");
        mHomingGoalEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Homing_Goal");
        mDriveOutputEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Output_Drive");
        mTurnOutputEntry = NetworkTableInstance.getDefault().getEntry("/"+swervePodID+"/Output_Turn");

    }
    
}
