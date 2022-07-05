package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    private Drivetrain mDrivetrain;
    private double mXSpeed;
    private double mYSpeed;
    private double mRoation;
    private boolean mFieldOriented;

    public Drive ( Drivetrain drivetrain, double xSpeed, double ySpeed, double rotation, boolean fieldOriented ) {
        mDrivetrain = drivetrain;
        mXSpeed = xSpeed;
        mYSpeed = ySpeed;
        mRoation = rotation;
        mFieldOriented = fieldOriented;
        addRequirements( mDrivetrain );
    }

    @Override
    public void execute() {
        System.out.println("xYpeed: "+mXSpeed+" mYSpeed: "+mYSpeed+" rotation: "+ mRoation);
        mDrivetrain.Drive( mXSpeed, mYSpeed, mRoation, mFieldOriented );
    }

    @Override
    public void end( boolean interrupted ) {
        mDrivetrain.Drive( 0, 0, 0, false );
    }

}