package frc.robot;

public class Constants {
    
    public static final double maxSpeed = 0.9;

    public static final double modulekP = 1/90;

    public static final double driveRevsToMeters = 4 * Math.PI / (39.37 * 8.14)  * 1.98;
    public static final double driveRPS2MPS = driveRevsToMeters;
}
