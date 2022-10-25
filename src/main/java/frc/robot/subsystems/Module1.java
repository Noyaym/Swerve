package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Module;

public class Module1 extends SubsystemBase {

    private final Module module;

    public Module1() {
        module = new Module(false, Constants.ModuleConst.CAN_PORT_NUM1,
        Constants.ModuleConst.mVel_PORT_NUM1, Constants.ModuleConst.mAngle_PORT_NUM1);


        setDefaultCommand(new RunCommand(()-> module.setVel(0), this)
        .alongWith(new RunCommand(()->module.setAngle(0), this)));
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("target vel", 0);
        SmartDashboard.putNumber("target angle", 0);

        SmartDashboard.putData("set vel", new RunCommand(()->
        module.setVel(SmartDashboard.getNumber("target vel", 0)), this));

        SmartDashboard.putData("set angle", new RunCommand(()->
        module.setAngle(SmartDashboard.getNumber("target angle", 0)), this));


    }
    
}
