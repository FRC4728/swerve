package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.HARDWARE;
import frc.robot.drivers.SwerveModule;
import frc.robot.utils.SwerveDriveSignal;
import frc.robot.UpdateManager;

public class Drivetrain implements Subsystem, UpdateManager.Updatable {

    // State enumerations
    private enum State_t {
        Homing { @Override public String toString() { return "Homing"; } },
        Teleop { @Override public String toString() { return "Teleop"; } },
        Following { @Override public String toString() { return "Following"; } },
        Idle { @Override public String toString() { return "Idle"; } },
    }
    private State_t mState;

    // The physical position of the swerve modules relative to the center of
    // the robot are used for the swerve drive kinematics which in turn is
    // used for the odometry.
    private final Translation2d mFrontLeftLocation = 
        new Translation2d( Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                           Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 );
    private final Translation2d mFrontRightLocation = 
        new Translation2d( Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                           -Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 );
    private final Translation2d mRearLeftLocation = 
        new Translation2d( -Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                           Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 );
    private final Translation2d mRearRightLocation = 
       new Translation2d( -Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                          -Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 );
    private final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics( mFrontLeftLocation, mFrontRightLocation,
                                   mRearLeftLocation, mRearRightLocation);
    
    private final SwerveModule[] mModules;
    private final SwerveDriveOdometry mOdometry;
    private final Gyro mGyro;
    private SwerveDriveSignal mDriveSignal;

    private final NetworkTableEntry mOdometryXEntry;
    private final NetworkTableEntry mOdometryYEntry;
    private final NetworkTableEntry mOdometryHeadingEntry;

    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//

    public void HomeModules () {
        State_t state = GetState();
        if ( state != State_t.Idle ) {
            SetState( State_t.Homing );
        } else {
            DriverStation.reportWarning( "Cannot zero modules from state: "+state, false);
        }
    }


    /**
     * Teleop interface to drive the robot.
     * 
     * TODO: The field relative driving requires an accurate measure of the
     * robots heading. Need to come up with a process to ensure accurate
     * heading measurements througout the match (like when to do heading
     * resets, and etc.).
     *
     * @param xSpeed Speed of the robot in the x direction (meters per second).
     * @param ySpeed Speed of the robot in the y direction (meters per second).
     * @param rot Rate of the robot rotation (radians per second).
     * @param fieldOriented Whether the provided x and y speeds are relative to
     * the field or the robot.
     */
    public void Drive ( double xSpeed, double ySpeed, double rotation, boolean fieldOriented ) {
        SetDriveSignal( new SwerveDriveSignal( xSpeed, ySpeed, rotation, fieldOriented ) );
    }


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    private synchronized State_t GetState () {
        return mState;
    }

    private synchronized void SetState ( State_t state ) {
        mState = state;
    }

    // private synchronized SwerveDriveSignal GetDriveSignal () {
    //     return mDriveSignal;
    // }

    private synchronized void SetDriveSignal ( SwerveDriveSignal driveSignal ) {
        mDriveSignal = driveSignal;
    }

    private synchronized Pose2d GetPose () {
        return mOdometry.getPoseMeters();

    }

    /**
     * This function will make periodic updates to the state of the swerve
     * modules. The order of the pods MUST match the order of the constructor
     * of the SwerveDriveKinematics.
     */  
    private synchronized void UpdateOdometry ( double time ) {
        mOdometry.updateWithTime( time, 
                                  GetGyroHeading(), 
                                  mModules[0].GetState(),
                                  mModules[1].GetState(), 
                                  mModules[2].GetState(), 
                                  mModules[3].GetState());
    }


    private void UpdateModules ( SwerveDriveSignal driveSignal ) {
        SwerveModuleState[] swerveModuleStates;
        if ( driveSignal.GetIsFieldOriented() ) {
            swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds( driveSignal.GetXSpeed(),
                                                       driveSignal.GetYSpeed(),
                                                       driveSignal.GetRotation(),
                                                       GetGyroHeading() ) );
        } else {
            swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds( driveSignal.GetXSpeed(),
                                   driveSignal.GetYSpeed(),
                                   driveSignal.GetRotation() ) );
        }
    
        // Clamp the wheel speeds to what the driver/robot are capable of
        // handling.
        SwerveDriveKinematics.normalizeWheelSpeeds( swerveModuleStates, DRIVETRAIN.MAX_DRIVE_VELOCITY_MPS );
        
        for (int i = 0; i < mModules.length; i++) {
            mModules[i].SetState( swerveModuleStates[i] );
        }
    }




    private Rotation2d GetGyroHeading () {
        return mGyro.getRotation2d();
    }


    // /** 
    //  * Reset the robots heading to 0.
    //  * */
    // private void ResetHeading () {
    //     mGyro.reset();
    // }


    // public void CalibrateGyro () {
    //     mGyro.calibrate();
    // }

    // /**
    //  * Get the robots heading.
    //  *
    //  * @return the robots heading in degrees (-180 to 180)
    //  */
    // private double GetHeading() {
    //     return mGyro.getRotation2d().getDegrees();
    // }

    // /**
    //  * Get the robots rate of turn.
    //  *
    //  * @return The turn rate of the robot (degrees per second)
    //  */
    // private double GetTurnRate() {
    //     return mGyro.getRate();
    // }


    //-------------------------------------------------------------------------------------------//
    /*                            CONSTRUCTOR AND PERIODIC METHODS                               */
    //-------------------------------------------------------------------------------------------//


    /**
     * The constructor for the Drivetrain class.
     */  
    public Drivetrain () {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        SwerveModule mFrontLeft =
            new SwerveModule( "FrontLeft", 
                              tab.getLayout("Front Left Module", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4),
                              HARDWARE.FRONT_LEFT_DRIVE_MOTOR_ID, 
                              HARDWARE.FRONT_LEFT_TURN_MOTOR_ID,
                              HARDWARE.FRONT_LEFT_QUAD_A_DIO_CHANNEL,
                              HARDWARE.FRONT_LEFT_QUAD_B_DIO_CHANNEL, 
                              HARDWARE.FRONT_LEFT_PWM_DIO_CHANNEL,
                              false,
                              true );
        SwerveModule mFrontRight =
            new SwerveModule( "FrontRight",
                              tab.getLayout("Front Right Module", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4),
                              HARDWARE.FRONT_RIGHT_DRIVE_MOTOR_ID,
                              HARDWARE.FRONT_RIGHT_TURN_MOTOR_ID,
                              HARDWARE.FRONT_RIGHT_QUAD_A_DIO_CHANNEL,
                              HARDWARE.FRONT_RIGHT_QUAD_B_DIO_CHANNEL,
                              HARDWARE.FRONT_RIGHT_PWM_DIO_CHANNEL,
                              true,
                              true );
        SwerveModule mRearLeft =
            new SwerveModule( "RearLeft",
                              tab.getLayout("Rear Left Module", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 4),
                              HARDWARE.REAR_LEFT_DRIVE_MOTOR_ID,
                              HARDWARE.REAR_LEFT_TURN_MOTOR_ID,
                              HARDWARE.REAR_LEFT_QUAD_A_DIO_CHANNEL,
                              HARDWARE.REAR_LEFT_QUAD_B_DIO_CHANNEL,
                              HARDWARE.REAR_LEFT_PWM_DIO_CHANNEL,
                              false,
                              true );
        SwerveModule mRearRight =
            new SwerveModule( "RearRight",
                              tab.getLayout("Rear Right Module", BuiltInLayouts.kList).withPosition(8, 0).withSize(2, 4),
                              HARDWARE.REAR_RIGHT_DRIVE_MOTOR_ID,
                              HARDWARE.REAR_RIGHT_TURN_MOTOR_ID,
                              HARDWARE.REAR_RIGHT_QUAD_A_DIO_CHANNEL,
                              HARDWARE.REAR_RIGHT_QUAD_B_DIO_CHANNEL,
                              HARDWARE.REAR_RIGHT_PWM_DIO_CHANNEL,
                              true,
                              true );
        mState = State_t.Idle;
        mModules = new SwerveModule[]{mFrontLeft, mFrontRight, mRearLeft, mRearRight};
        mGyro = new ADXRS450_Gyro();
        mOdometry = new SwerveDriveOdometry( kDriveKinematics, new Rotation2d( 0.0 ) );
        mOdometryXEntry = tab.add("X", 0.0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();
        mOdometryYEntry = tab.add("Y", 0.0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();
        mOdometryHeadingEntry = tab.add("Heading", 0.0)
            .withPosition(0, 2)
            .withSize(1, 1)
            .getEntry();


    }

    /**
     * This method is called periodically by the main program thread.
     */  
    @Override 
    public void periodic() {
        Pose2d pose = GetPose();
        mOdometryXEntry.setDouble( pose.getX() );
        mOdometryYEntry.setDouble( pose.getY() );
        mOdometryHeadingEntry.setDouble( pose.getRotation().getDegrees() );
    }

    /**
     * This method is called periodically by the update manager thread.
     */  
    @Override
    public void Update ( double time, double dt ) {
        
        // Read the sensor inputs
        UpdateOdometry( time );

        // Set the motor outputs and update state
        State_t state = GetState();
        switch ( state ) {
        case Homing:
            for (int i = 0; i < mModules.length; i++) {
                mModules[i].SetState( new SwerveModuleState( 0.0, new Rotation2d( -mModules[i].GetTurnAbsolutePosition() ) ) );
            }
            boolean everyoneAtGoal = true;
            for ( int i = 0; i < mModules.length; i++ ) {
                everyoneAtGoal = everyoneAtGoal && mModules[i].GetTurnControllerAtGoal();
            }
            if ( everyoneAtGoal ) {
                for ( int i = 0; i < mModules.length; i++ ) {
                    mModules[i].ResetEncoders();
                }
                SetState( State_t.Teleop );
                DriverStation.reportWarning( "Successfully homed all drivetrain wheels", false);
            }

            break;

        // case Teleop:
        //     // UpdateModules( GetDriveSignal() );
        //     UpdateModules( new SwerveDriveSignal( 0, 0, 0, false ) );
        //     break;

        // case Following:
        //     UpdateModules( new SwerveDriveSignal( 0, 0, 0, false ) );
        //     break;

        case Idle:
            for ( int i = 0; i < mModules.length; i++ ) {
                mModules[i].NullOutput();
            }
            break;

        default:
            DriverStation.reportWarning( "Unknown drivetrain state: "+state, false);
            for ( int i = 0; i < mModules.length; i++ ) {
                mModules[i].NullOutput();
            }

        }

    }


}
