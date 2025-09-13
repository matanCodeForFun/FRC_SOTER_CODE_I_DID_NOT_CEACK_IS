package frc.demacia.utils.Motors;

public class SparkFlexConfig extends BaseMotorConfig<SparkFlexConfig> {

    /** 
     * Constructor
     * @param id - canbus ID
     * @param name - name of motor for logging
     */
    public SparkFlexConfig(int id, String name) {
        super(id, name);
        motorType = MotorControllerType.SparkFlex;
    }

    public SparkFlexConfig(int id, String name, SparkFlexConfig config) {
        this(id, name);
        copyBaseFields(config);
    }
}