package frc.demacia.utils.Motors;

public class TalonFXSConfig extends BaseMotorConfig<TalonFXSConfig> {

    /** 
     * Constructor
     * @param id - canbus ID
     * @param name - name of motor for logging
     */
    public TalonFXSConfig(int id, String name) {
        super(id, name);
        motorType = MotorControllerType.TalonFX;
    }

    public TalonFXSConfig(int id, String name, TalonFXSConfig config) {
        this(id, name);
        copyBaseFields(config);
    }
}