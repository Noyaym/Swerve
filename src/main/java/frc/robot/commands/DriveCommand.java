package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassiConst;
import frc.robot.subsystems.Chassis;

public class DriveCommand extends CommandBase {

    private Chassis chassis;

    public DriveCommand(Chassis ch) {
        this.chassis = ch;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        double vx = chassis.getJoystickX(RobotContainer.getJoystickXY());
        double vy = chassis.getJoystickY(RobotContainer.getJoystickXY());
        double ang = chassis.getJoystickAngle(RobotContainer.getJoystickDirection());
        SwerveModuleState[] sms = chassis.getSwerveState(vx, vy, ang);
        chassis.setModules(sms);
    }
    
}
