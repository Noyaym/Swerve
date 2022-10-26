package frc.robot.subsystems;

import java.security.PublicKey;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Module;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {

    private final Module m1;
    private final Module m2;
    private final Module m3;
    private final Module m4;

    private final SwerveDriveKinematics swerveK;
    private SwerveDriveOdometry swerveOd;
    private final PIDController PIDangle2radPerSec;
    Field2d f2d;


    public Chassis() {
        m1 = new Module(false, Constants.ModuleConst.CAN_PORT_NUM1,
        Constants.ModuleConst.mVel_PORT_NUM1, Constants.ModuleConst.mAngle_PORT_NUM1);

        m2 = new Module(false, Constants.ModuleConst.CAN_PORT_NUM2,
        Constants.ModuleConst.mVel_PORT_NUM2, Constants.ModuleConst.mAngle_PORT_NUM2);

        m3 = new Module(false, Constants.ModuleConst.CAN_PORT_NUM3,
        Constants.ModuleConst.mVel_PORT_NUM3, Constants.ModuleConst.mAngle_PORT_NUM3);

        m4 = new Module(false, Constants.ModuleConst.CAN_PORT_NUM4,
        Constants.ModuleConst.mVel_PORT_NUM4, Constants.ModuleConst.mAngle_PORT_NUM4);

        swerveK = new SwerveDriveKinematics(Constants.ChassiConst.wheelsMeters);

        PIDangle2radPerSec = new PIDController(Constants.ChassiConst.a2r_Kp,
        Constants.ChassiConst.a2r_Ki, Constants.ChassiConst.a2r_Kd);

        swerveOd = new SwerveDriveOdometry(swerveK, getRotation2d(getJyroPosition(RobotContainer.getGyro())));
        f2d = new Field2d();

        SmartDashboard.putData("Field", getField2d());

    }

    public SwerveModuleState[] getSwerveState(double vx, double vy, double desiredAngle) {
        double dif = desiredAngle - getJyroPosition(RobotContainer.getGyro());
        double radPerSec = PIDangle2radPerSec.calculate(dif);
        return getModuleStates(vx, vy, radPerSec, getRotation2d(getJyroPosition(RobotContainer.getGyro())));

    }


    public SwerveModuleState[] getModuleStates(double vx, double vy, double radPerSec, Rotation2d angle) {
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, radPerSec, angle);
        SwerveModuleState[] sms = swerveK.toSwerveModuleStates(cspeeds);
        for (int i=0; i<4; i++) {
            sms[i] = SwerveModuleState.optimize(sms[i], angle); ///this is NOT correct
        }
        return sms;
    }




    public double getJyroPosition (PigeonIMU gyro) {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d(double angle) {
        return new Rotation2d(angle);
    }

    public double getJoystickX (Joystick j) {
        double x = -j.getX();
        double val;
        if ((x<Constants.Buttons.j_RANGE) && (x>-Constants.Buttons.j_RANGE)) {
            val=0.0;
        }
        else {
            val = Math.signum(x)*((1/1.1)*Math.pow(x, 2)+(1-1/1.1));

        }
        return val;
    }

    public double getJoystickY (Joystick j) {
        double y = -j.getY();
        double val;
        if ((y<Constants.Buttons.j_RANGE) && (y>-Constants.Buttons.j_RANGE)) {
            val=0.0;
        }
        else {
            val = Math.signum(y)*((1/1.1)*Math.pow(y, 2)+(1-1/1.1));

        }
        return val;
    }

    public double getJoystickAngle (Joystick j) {
        double y = j.getY();
        double x = j.getX();
        double angle  = Math.atan(y/x);
        double val;
        if ((angle<Constants.Buttons.j_RANGE) && (angle>-Constants.Buttons.j_RANGE)) {
            val=0.0;
        }
        else {
            val = Math.signum(angle)*((1/1.1)*Math.pow(angle, 2)+(1-1/1.1));

        }
        return val;

    }

    public void setModules (SwerveModuleState[] sms) {
        m1.setAngle(sms[0].angle.getDegrees());
        m1.setVel(sms[0].speedMetersPerSecond);

        m2.setAngle(sms[1].angle.getDegrees());
        m2.setVel(sms[1].speedMetersPerSecond);

        m3.setAngle(sms[2].angle.getDegrees());
        m3.setVel(sms[2].speedMetersPerSecond);

        m4.setAngle(sms[3].angle.getDegrees());
        m4.setVel(sms[3].speedMetersPerSecond);

    }

    public Pose2d getPose2d() {
        return swerveOd.getPoseMeters();

    }

    public Field2d getField2d() {
        return f2d;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        double vx = getJoystickX(RobotContainer.getJoystickXY());
        double vy = getJoystickY(RobotContainer.getJoystickXY());
        double ang = getJoystickAngle(RobotContainer.getJoystickDirection());
        SwerveModuleState[] sms = getSwerveState(vx, vy, ang);
        swerveOd.update(getRotation2d(getJyroPosition(RobotContainer.getGyro())), sms);
        f2d.setRobotPose(getPose2d());
        
        
        
    }



}