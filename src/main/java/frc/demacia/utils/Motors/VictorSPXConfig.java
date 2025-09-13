package frc.demacia.utils.Motors;

public class VictorSPXConfig extends BaseMotorConfig<VictorSPXConfig> {

    /** 
     * Constructor
     * @param id - canbus ID
     * @param name - name of motor for logging
     */
    public VictorSPXConfig(int id, String name) {
        super(id, name);
        motorType = MotorControllerType.VictorSPX;
    }

    public VictorSPXConfig(int id, String name, VictorSPXConfig config) {
        this(id, name);
        copyBaseFields(config);
    }
}