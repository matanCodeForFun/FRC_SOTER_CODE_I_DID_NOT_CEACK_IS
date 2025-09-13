package frc.demacia.utils.Motors;

public class TalonSRXConfig extends BaseMotorConfig<TalonSRXConfig> {

    /** 
     * Constructor
     * @param id - canbus ID
     * @param name - name of motor for logging
     */
    public TalonSRXConfig(int id, String name) {
        super(id, name);
        motorType = MotorControllerType.TalonSRX;
    }

    public TalonSRXConfig(int id, String name, TalonSRXConfig config) {
        this(id, name);
        copyBaseFields(config);
    }
}