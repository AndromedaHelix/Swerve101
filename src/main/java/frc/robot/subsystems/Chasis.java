package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Chasis extends SubsystemBase{
    
     Module frenteIzq, frenteDer, atrasIzq, atrasDer;
    Translation2d fITranslation = new Translation2d(0.5, 0.5);
    Translation2d fDTranslation = new Translation2d(-0.5,-0.5);
    Translation2d aITranslation = new Translation2d(0.5, -0.5);
    Translation2d aDTranslation = new Translation2d(-0.5, 0.5);

    Pigeon2 gyro = new Pigeon2(0);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(fITranslation,fDTranslation, aITranslation, aDTranslation);

     SwerveModulePosition[] positions = {frenteIzq.getPosition(), frenteDer.getPosition(), 
        atrasIzq.getPosition(), atrasDer.getPosition()};

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    public Chasis(){
        frenteDer = new Module(0, 0, 0, Constants.modulekP, 0, 0);
        frenteIzq = new Module(0, 0, 0, Constants.modulekP, 0, 0);
        atrasDer = new Module(0, 0, 0, Constants.modulekP, 0, 0);
        atrasIzq = new Module(0, 0, 0, Constants.modulekP, 0, 0);
        
    }

    public void setChassisSpeeds(double xSpeed, double ySpeed, double zSpeed){

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));

        setModuleStates(states);
    }

    public void setFieldOrientedSpeed(double xSpeed, double ySpeed, double zSpeed){
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d());

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

        setModuleStates(states);

    }

    public void setModuleStates(SwerveModuleState[] states){
        frenteIzq.setDesiredState(states[0]);
        frenteDer.setDesiredState(states[1]);
        atrasIzq.setDesiredState(states[2]);
        atrasDer.setDesiredState(states[3]);
    }

    public double getAngle(){
        return gyro.getAngle();
    }

    public Rotation2d getRotation2d(){
        return new Rotation2d(getAngle());
    }

    public Pose2d getPose2d(){
        return odometry.getPoseMeters();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("gyro angle",getAngle());
        SmartDashboard.putNumber("pose x", getPose2d().getX());

        positions[0] = frenteIzq.getPosition();
        positions[1] = frenteDer.getPosition();
        positions[2] = atrasIzq.getPosition();
        positions[3] = atrasDer.getPosition();
        odometry.update(getRotation2d(), positions);
    }


}
