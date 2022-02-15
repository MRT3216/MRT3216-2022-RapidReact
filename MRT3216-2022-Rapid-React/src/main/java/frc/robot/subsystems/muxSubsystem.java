package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.settings.*;


public class muxSubsystem extends SubsystemBase{
    private final int muxAddress;
    private final I2C mux;
    private final I2C.Port port = RobotMap.ROBOT.SENSORS.MUX_PORT;


    public muxSubsystem (int addr) {
        muxAddress = addr;
        mux = new I2C(port, addr);
    }
    public muxSubsystem() {
        //default address for the TCA9548A
        this(0x70); //fine
    }

    public void setIndex (int index) {
        this.mux.write(this.muxAddress, 1 << index);
    }

    public I2C.Port getPort () {
        return this.port;
    }
}