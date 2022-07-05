package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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

    private final SwerveModule[] mModules;
    private final SwerveDriveKinematics mDriveKinematics;
    private final SwerveDriveOdometry mOdometry;
    private final Gyro mGyro;
    private SwerveDriveSignal mDriveSignal = new SwerveDriveSignal(0, 0, 0, false);
    private double[] mHomes = new double[]{0, 0, 0, 0};
    private State_t mState = State_t.Homing;
    private State_t mNextState = State_t.Homing;
    
    private final NetworkTableEntry mOdometryXEntry;
    private final NetworkTableEntry mOdometryYEntry;
    private final NetworkTableEntry mOdometryHeadingEntry;
    private final NetworkTableEntry mStateEntry;


    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Teleop interface to drive the robot.
     * @param xSpeed Speed of the robot in the x direction (meters per second).
     * @param ySpeed Speed of the robot in the y direction (meters per second).
     * @param rot Rate of the robot rotation (radians per second).
     * @param fieldOriented Whether the provided x and y speeds are relative to
     * the field or the robot.
     */
    public void Drive ( double xSpeed, double ySpeed, double rotation, boolean fieldOriented ) {
        SetDriveSignal( new SwerveDriveSignal( xSpeed, ySpeed, rotation, fieldOriented ) );
    }

    // public void ResetHeading () {
    //     mGyro.reset();
    // }


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    private synchronized State_t GetState () {
        return mState;
    }

    private synchronized void SetState ( State_t state ) {
        mState = state;
    }

    private synchronized SwerveDriveSignal GetDriveSignal () {
        return mDriveSignal;
    }

    private synchronized void SetDriveSignal ( SwerveDriveSignal driveSignal ) {
        mDriveSignal = driveSignal;
    }

    private synchronized Pose2d GetPose () {
        return mOdometry.getPoseMeters();
    }

    private Rotation2d GetGyroHeading () {
        return mGyro.getRotation2d();
    }

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
            swerveModuleStates = mDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds( driveSignal.GetXSpeed(),
                                                       driveSignal.GetYSpeed(),
                                                       driveSignal.GetRotation(),
                                                       GetGyroHeading() ) );
        } else {
            // double speed = new ChassisSpeeds( driveSignal.GetXSpeed(),
            //                                     driveSignal.GetYSpeed(),
            //                                     driveSignal.GetRotation() );
            swerveModuleStates = mDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds( driveSignal.GetXSpeed(),
                                   driveSignal.GetYSpeed(),
                                   driveSignal.GetRotation() ) );
        }
    
        SwerveDriveKinematics.normalizeWheelSpeeds( swerveModuleStates, DRIVETRAIN.MAX_DRIVE_VELOCITY_MPS );
        
        for (int i = 0; i < mModules.length; i++) {
            mModules[i].SetState( swerveModuleStates[i] );
        }
    }


    //-------------------------------------------------------------------------------------------//
    /*                            CONSTRUCTOR AND PERIODIC METHODS                               */
    //-------------------------------------------------------------------------------------------//


    /**
     * The constructor for the Drivetrain class.
     */  
    public Drivetrain () {
        ShuffleboardTab tab = Shuffleboard.getTab( "Drivetrain" );
        SwerveModule mFrontLeft =
            new SwerveModule( tab.getLayout( "Front Left Module",
                                             BuiltInLayouts.kList )
                                             .withPosition( 2, 0 )
                                             .withSize( 2, 4 ),
                              HARDWARE.FRONT_LEFT_DRIVE_MOTOR_ID, 
                              HARDWARE.FRONT_LEFT_TURN_MOTOR_ID,
                              DRIVETRAIN.FRONT_LEFT_ZERO_RAD,
                              HARDWARE.FRONT_LEFT_QUAD_A_DIO_CHANNEL,
                              HARDWARE.FRONT_LEFT_QUAD_B_DIO_CHANNEL, 
                              HARDWARE.FRONT_LEFT_PWM_DIO_CHANNEL,
                              false,
                              true );
        SwerveModule mFrontRight =
            new SwerveModule( tab.getLayout( "Front Right Module",
                                             BuiltInLayouts.kList )
                                             .withPosition( 4, 0 )
                                             .withSize( 2, 4 ),
                              HARDWARE.FRONT_RIGHT_DRIVE_MOTOR_ID,
                              HARDWARE.FRONT_RIGHT_TURN_MOTOR_ID,
                              DRIVETRAIN.FRONT_RIGHT_ZERO_RAD,
                              HARDWARE.FRONT_RIGHT_QUAD_A_DIO_CHANNEL,
                              HARDWARE.FRONT_RIGHT_QUAD_B_DIO_CHANNEL,
                              HARDWARE.FRONT_RIGHT_PWM_DIO_CHANNEL,
                              true,
                              true );
        SwerveModule mRearLeft =
            new SwerveModule( tab.getLayout( "Rear Left Module",
                                             BuiltInLayouts.kList )
                                             .withPosition( 6, 0 )
                                             .withSize( 2, 4 ),
                              HARDWARE.REAR_LEFT_DRIVE_MOTOR_ID,
                              HARDWARE.REAR_LEFT_TURN_MOTOR_ID,
                              DRIVETRAIN.REAR_LEFT_ZERO_RAD,
                              HARDWARE.REAR_LEFT_QUAD_A_DIO_CHANNEL,
                              HARDWARE.REAR_LEFT_QUAD_B_DIO_CHANNEL,
                              HARDWARE.REAR_LEFT_PWM_DIO_CHANNEL,
                              false,
                              true );
        SwerveModule mRearRight =
            new SwerveModule( tab.getLayout( "Rear Right Module",
                                             BuiltInLayouts.kList )
                                             .withPosition( 8, 0 )
                                             .withSize( 2, 4 ),
                              HARDWARE.REAR_RIGHT_DRIVE_MOTOR_ID,
                              HARDWARE.REAR_RIGHT_TURN_MOTOR_ID,
                              DRIVETRAIN.REAR_RIGHT_ZERO_RAD,
                              HARDWARE.REAR_RIGHT_QUAD_A_DIO_CHANNEL,
                              HARDWARE.REAR_RIGHT_QUAD_B_DIO_CHANNEL,
                              HARDWARE.REAR_RIGHT_PWM_DIO_CHANNEL,
                              true,
                              true );
        mModules = new SwerveModule[]{ mFrontLeft,
                                       mFrontRight,
                                       mRearLeft,
                                       mRearRight };
        mDriveKinematics = new SwerveDriveKinematics( DRIVETRAIN.FRONT_LEFT_LOCATION,
                                                      DRIVETRAIN.FRONT_RIGHT_LOCATION,
                                                      DRIVETRAIN.REAR_LEFT_LOCATION,
                                                      DRIVETRAIN.REAR_RIGHT_LOCATION );
        mOdometry = new SwerveDriveOdometry( mDriveKinematics, new Rotation2d( 0.0 ) );
        mGyro = new ADXRS450_Gyro();
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
        mStateEntry = tab.add("State", State_t.Idle.toString() )
            .withPosition(0, 3)
            .withSize(1, 1)
            .getEntry();


        // Make sure the module isn't too far off. The homing routine can
        // correct for ~180 degrees of error in the initial setup.
        for ( int i = 0; i < mModules.length; i++ ) {
            double zero = mModules[i].GetHomePosition();
            double absolutePosition = mModules[i].GetTurnAbsolutePosition();
            double home = zero - absolutePosition;
            double margin = Math.PI / 2;
            if ( zero >= margin && zero <= 2 * Math.PI - margin ) {
                if ( !( ( absolutePosition >= zero - margin ) && 
                        ( absolutePosition <= zero + margin ) ) ) {
                    DriverStation.reportError( "Cannot zero modules: module "+i+"distance to home "+
                    Units.radiansToDegrees( home ), false);
                } else {
                    mHomes[i] = home;
                }
            } else if ( zero < margin ) {
                if ( !( ( absolutePosition > zero + 2.0 * Math.PI - margin ) || 
                        ( absolutePosition < zero + margin ) ) ) {
                    DriverStation.reportError( "Cannot zero modules: module "+i+"distance to home "+
                    Units.radiansToDegrees( home ), false);
                } else {
                    mHomes[i] = home;
                }
            } else {
                if ( !( ( absolutePosition > zero - margin ) || 
                        ( absolutePosition < zero + margin - 2.0 * Math.PI ) ) ) {
                    DriverStation.reportError( "Cannot zero modules: module "+i+"distance to home "+
                    Units.radiansToDegrees( home ), false);
                } else {
                    mHomes[i] = home;
                }
            }
        }
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
        mStateEntry.setString( GetState().toString() );
        
    }

    /**
     * This method is called periodically by the update manager thread.
     */  
    @Override
    public void Update ( double time, double dt ) {
        
        // Update the subsystem state
        State_t currentState = GetState();
        if ( !currentState.equals( mNextState ) ) {
            switch( currentState ) {
                case Homing:
                for ( int i = 0; i < mModules.length; i++ ) {
                    mModules[i].ResetEncoders();
                }
                break;

                case Teleop:
                // for (int i = 0; i < mModules.length; i++) {
                //     mModules[i].SetNullOutput();
                // }                
                break;

                case Following:
                for (int i = 0; i < mModules.length; i++) {
                    mModules[i].SetNullOutput();
                }                
                break;

                case Idle:
                for (int i = 0; i < mModules.length; i++) {
                    mModules[i].SetNullOutput();
                }                
                break;
            }
        }
        currentState = mNextState;
        SetState( currentState );


        // Read the sensor inputs
        UpdateOdometry( time );

        // Set the motor outputs based on the current state
        switch ( currentState ) {
            case Homing:
            for (int i = 0; i < mModules.length; i++) {
                mModules[i].SetState( new SwerveModuleState( 0.0, new Rotation2d( mHomes[i] ) ) );
            }              
            boolean everyoneAtGoal = true;
            for ( int i = 0; i < mModules.length; i++ ) {
                everyoneAtGoal = everyoneAtGoal && mModules[i].GetTurnControllerAtGoal();
            }
            if ( everyoneAtGoal ) {
                // mNextState = State_t.Idle;
                mNextState = State_t.Teleop;
                // This will allow the motors/encoders to stop moving before
                // resetting the encoders
                for (int i = 0; i < mModules.length; i++) {
                    mModules[i].SetNullOutput();
                }
                DriverStation.reportWarning( "Successfully homed all drivetrain wheels", false);
            }
            break;

            case Teleop:
            UpdateModules( GetDriveSignal() );
            // for ( int i = 0; i < mModules.length; i++ ) {
            //     mModules[i].SetNullOutput();
            // }
            break;

            case Following:
            for ( int i = 0; i < mModules.length; i++ ) {
                mModules[i].SetNullOutput();
            }
            break;

            case Idle:
            for ( int i = 0; i < mModules.length; i++ ) {
                mModules[i].SetNullOutput();
            }
            break;

        }

    }

}

