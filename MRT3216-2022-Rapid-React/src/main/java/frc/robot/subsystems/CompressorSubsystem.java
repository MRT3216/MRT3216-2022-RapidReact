/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {
    private final Compressor compressor;

    /**
     * Creates a new CompressorSubsystem.
     */
    public CompressorSubsystem() {
        this.compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void start() {
        this.compressor.enableDigital();
    }

    public void stop() {
        this.compressor.disable();
    }
}
