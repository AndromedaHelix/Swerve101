package frc.robot.commands;


import frc.robot.subsystems.Chasis;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;


public class Manejo  extends Command{
    Chasis chasis;
    Supplier<Double> xSpeed, ySpeed, zSpeed;

    public Manejo(Chasis chasis, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> zSpeed){
        addRequirements(chasis);
        this.chasis = chasis;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;
    }

    @Override
    public void execute(){
        chasis.setFieldOrientedSpeed(xSpeed.get(), ySpeed.get(), zSpeed.get());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
