package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.WPI_CANCoder;


public class SwerveModule {
    //da motors :)
private CANSparkMax driveMotor;
private CANSparkMax turningMotor;
//built in encoders
private RelativeEncoder driveEncoder;
private RelativeEncoder turningEncoder;
//to move angle motor
private PIDController turningPidController;
//absolute encoder so the wheel position can be kept constantly
//connect to the rio
private WPI_CANCoder absoluteEncoder;
//
private boolean absoluteEncoderReversed;
//offset position
//used to compensate for encoder error
private double absoluteEncoderOffsetRad;
private int turningMotorId;

public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
                    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                    this.absoluteEncoderReversed = absoluteEncoderReversed;
                    //cancoder as absolute;
                    this.absoluteEncoder = new WPI_CANCoder(0, "rio");
                    //changing cancoder from a default range of (0,360) to (-180, 180) cuz thats how tf it works
                    this.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
                    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
                    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
                    this.turningMotorId=turningMotorId;

                    driveEncoder = driveMotor.getEncoder();
                    //device number and canbus need to be changed.
                    turningEncoder = turningMotor.getEncoder();
                    
                    //Cancoder version of this???
                    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
                    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
                    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
                    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
                    

                    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
                    //turns PID controller into a circular system (tells it that -180 is right next to 179 (-180 and 180 cant both exist))
                    turningPidController.enableContinuousInput(-Math.PI,Math.PI);

                    resetEncoders();
                    }
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }
    public double getAbsoluteEncoderRad(){
        //how many percent of a full rotation it is currently reading
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= (Math.PI/180);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed? -1.0 : 1.0);
    }
    public double getAbsoluteEncoderReading(){
        return absoluteEncoder.getAbsolutePosition();
    }
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());

        
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));    
    }
    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;

        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        // ???
        SmartDashboard.putString("Swerve[" + turningMotorId+"] state", state.toString());
    }
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
