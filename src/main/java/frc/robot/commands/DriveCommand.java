package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassiConst;
import frc.robot.subsystems.Chassis;

public class DriveCommand extends CommandBase {

    private Chassis ch;

    public DriveCommand(Chassis ch) {
        this.ch = ch;
        addRequirements(ch);
    }

    @Override
    public void execute() {
        double vx = ch.getJoystickX(RobotContainer.getJoystickXY());
        double vy = ch.getJoystickY(RobotContainer.getJoystickXY());
        double ang = ch.getJoystickAngle(RobotContainer.getJoystickDirection());
        SwerveModuleState[] sms = ch.getSwerveState(vx, vy, ang);
        ch.setModules(sms);
    }
    
}
