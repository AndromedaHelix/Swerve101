package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

public class Autonomo extends Command{
     Chasis chasis;
     double xObjective, yObjective, zObjective;
     int estadoAutonomo = 0;

    PIDController drivePID = new PIDController(0,0,0);
    PIDController rotationPID = new PIDController(0,0,0);

     public Autonomo(Chasis chasis){
        this.chasis = chasis;
     }

      @Override
    public void initialize() {
        yObjective = 1;
        xObjective = chasis.getPose2d().getX();
        zObjective = 180;
    }

    @Override
    public void execute(){
        if(estadoAutonomo == 0){
            yObjective = 1;
            xObjective = chasis.getPose2d().getX();
            zObjective = 180;
        } else if (estadoAutonomo == 1){
            yObjective = 1;
            xObjective = 1;
            zObjective = 270;
        }



        double xSpeed = drivePID.calculate(chasis.getPose2d().getX(), xObjective);
        double ySpeed = drivePID.calculate(chasis.getPose2d().getY(), yObjective);
        double zSpeed = rotationPID.calculate(chasis.getPose2d().getRotation().getDegrees(), zObjective);



        if(Math.abs(xSpeed) < 0.1 && Math.abs(ySpeed) < 0.1 && Math.abs(zSpeed) < 0.1){
            estadoAutonomo++;
        }

        chasis.setFieldOrientedSpeed(xSpeed, ySpeed, zSpeed);

    }
}
