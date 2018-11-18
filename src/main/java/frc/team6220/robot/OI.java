package frc.team6220.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

    public final Joystick js1 = new Joystick(RobotMap.LeftJoystick);
    public final Joystick js2 = new Joystick(RobotMap.RightJoystick);
    public final XboxController dab = new XboxController(RobotMap.XboxController);

    public OI(){

    }

    public Joystick getLeftJoystick() {
        return js1;
    }

    public Joystick getRightJoystick() {
        return js2;
    }

    public XboxController getXboxController() {
        return dab;
    }
}
