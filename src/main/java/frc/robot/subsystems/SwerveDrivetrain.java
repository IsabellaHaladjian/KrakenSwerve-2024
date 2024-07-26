package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] m_swerveMods;
    public Pigeon2 m_gyro;

    public SwerveDrivetrain() {

        m_gyro = new Pigeon2(Constants.PIGEON_ID);
        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        m_swerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Mod0.constants),
            new SwerveModule(1, Constants.Mod1.constants),
            new SwerveModule(2, Constants.Mod2.constants),
            new SwerveModule(3, Constants.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SWERVE_KINEMATICS, getYaw(), getModulePositions());

        AutoBuilder.configureHolonomic(this::getPose, 
                                       this::resetOdometry, 
                                       this::getSpeeds, 
                                       this::driveRobotRelative, 
                                       Constants.pathFollowerConfig, 
                                       () -> {
                                            // Boolean supplier that controls when the path will be mirrored for the red alliance
                                            // This will flip the path being followed to the red side of the field.
                                            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                            var alliance = DriverStation.getAlliance();
                                            if (alliance.isPresent()) {
                                            return alliance.get() == DriverStation.Alliance.Red;
                                            }
                                            return false;
                                        }, 
                                        this);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    

        final ChassisSpeeds chassisSpeeds;
        
        if(fieldRelative) {

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation, 
                getYaw()
                );

        } else {

            chassisSpeeds = new ChassisSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation);
        } 

        final var swerveModuleStates = Constants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);

        for(SwerveModule mod : m_swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

    }  
    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED);
        
        for(SwerveModule mod : m_swerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }  

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = Constants.SWERVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);

    }

    public ChassisSpeeds getSpeeds() {

        return Constants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());

    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        m_gyro.setYaw(0);
    }

    public void adjustGyroZeroSourceSide() {

        m_gyro.setYaw(m_gyro.getYaw().getValueAsDouble() + Constants.AUTO_SOURCE_SIDE_START_OFFSET);

    }

     public void adjustGyroZeroAmpSide() {

        m_gyro.setYaw(m_gyro.getYaw().getValueAsDouble() + Constants.AUTO_AMP_SIDE_START_OFFSET);

    }

    public Rotation2d getYaw() {
        return (Constants.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw().getValue()) : Rotation2d.fromDegrees(m_gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : m_swerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : m_swerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}