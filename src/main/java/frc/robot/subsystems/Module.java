package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Clase que representa un módulo de swerve. Cada módulo tiene un motor de giro
 * y otro de tracción.
 * El módulo se encarga de controlar ambos motores para que el robot se mueva en
 * la dirección y velocidad
 * deseadas.
 * 
 * Esta clase es un ejemplo de cómo se puede implementar un módulo de swerve. No
 * es la única forma de hacerlo,
 * y no necesariamente es la mejor. Siéntanse libres de modificarla o
 * reemplazarla por otra implementación.
 * 
 * Reemplacen los motores y hardware por el que vayan a usar en su robot.
 */
public class Module {
    // Aquí se declaran los motores
    // Estos dos son motores Falcon
    TalonFX driveMotorFalcon;
    TalonFX turnMotorFalcon;

    // Estos dos son motores Neo
    CANSparkMax driveMotorNeo;
    CANSparkMax turnMotorNeo;

    CANcoder turnEncoder;

    PIDController turnPID;

    public Module(int driveMotorID, int turnMotorID, int cancoderID, double kP, double kI, double kD) {

        // Aquí se inicializan los motores. Se puede hacer con Falcon o con Neo, pero no
        // con ambos, por el ID. Se debe comentar una de las 4 líneas siguientes,
        // dependiendo del motor que usen
        driveMotorFalcon = new TalonFX(driveMotorID);
        turnMotorFalcon = new TalonFX(turnMotorID);

        driveMotorNeo = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotorNeo = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        
        // Se reinician los motores a su configuración de fábrica. Esto es una buena
        // práctica siempre que inicialicemos un motor
        //
        // La clase TalonFX tiene un método llamado "getConfigurator" que nos permite
        // configurar el motor. En este caso, se configura el motor de tracción y el de
        // giro con la configuración de fábrica.
        //
        // La clase TalonFXConfiguration representa toda la configuración de un motor
        // Falcon. Esto incluye inversión (si el motor gira al revés), el estado del
        // motor (Coast o Brake) y más. Para más información, revisen la documentación:
        // https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/configuration-guide.html
        //
        // Aquí un ejemplo de más cosas que se pueden configurar:
        // https://github.com/AndromedaHelix/AndromedaLib/blob/f4aa8047563c30b0d6c2bc25fa9d76b822489cad/src/main/java/com/andromedalib/andromedaSwerve/utils/AndromedaProfileConfig.java#L232
        driveMotorFalcon.getConfigurator().apply(new TalonFXConfiguration());
        turnMotorFalcon.getConfigurator().apply(new TalonFXConfiguration());

        driveMotorNeo.restoreFactoryDefaults();
        turnMotorNeo.restoreFactoryDefaults();

        // Se configura el encoder de giro. Esto es necesario para poder saber a qué
        // ángulo está apuntando la llanta cuando empieza el programa
        turnEncoder = new CANcoder(cancoderID);

        turnPID = new PIDController(kP, kI, kD);
    }

    public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(getDrivePosition(), getAngle());
    }    

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        setSpeed(desiredState);
        setAngle(desiredState);
    }

    public void setSpeed(SwerveModuleState desiredState) {
        driveMotorFalcon.set(desiredState.speedMetersPerSecond);
    }

    public void setAngle(SwerveModuleState desiredState) {
        double pidValue = turnPID.calculate(getAngle().getDegrees(), desiredState.angle.getDegrees());

        // Se aplica el valor calculado del PID
        driveMotorFalcon.set(pidValue);
        driveMotorFalcon.set(pidValue);
    }

    public Rotation2d getAngle() {
        // Diferentes formas de conseguir el ángulo. Depende de su hardware y cómo
        // quieran controlarlo.

        // Algo importante es que en caso de trabajar directamente con el encoder de los
        // motores, se debe tener en cuenta la relación de engranes entre el motor y la
        // llanta, para poder calcular el ángulo real de la llanta.

        // Esto se puede evitar usando un encoder externo, como el CANcoder, que ya
        // tiene una relaciín 1:1 con la llanta para los módulos MK4
        double value = turnMotorFalcon.getPosition().getValueAsDouble();
        value = turnMotorNeo.getEncoder().getPosition();
        value = turnEncoder.getAbsolutePosition().getValueAsDouble();
        return Rotation2d.fromDegrees(value);
    }

    public SwerveModuleState getSwerveState(){
        return new SwerveModuleState(getDriveVelocity(), getAngle());
     }

    public double getDriveVelocity(){
        return driveMotorFalcon.getVelocity().getValue() * Constants.driveRPS2MPS;
    }

    public double getDrivePosition(){
        double position;

        
        position = driveMotorFalcon.getPosition().getValue();
        position = driveMotorNeo.getEncoder().getPosition();

        position *= Constants.driveRevsToMeters;
        
        return position;
    }
}
