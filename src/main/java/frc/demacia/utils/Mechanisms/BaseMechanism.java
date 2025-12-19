package frc.demacia.utils.Mechanisms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.LookUpTable;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;
import frc.demacia.utils.Motors.MotorInterface;
import frc.demacia.utils.Sensors.SensorInterface;

/**
 * Base class for all robot mechanisms.
 * 
 * <p>Provides common functionality for managing motors and sensors with
 * trigger-based conditional control.</p>
 * 
 * <p><b>Features:</b></p>
 * <ul>
 *   <li>Trigger-based conditional actions</li>
 *   <li>Automatic safety stops</li>
 *   <li>Motor and sensor management</li>
 *   <li>Electronics health checking</li>
 * </ul>
 * 
 * <p><b>Example Usage:</b></p>
 * <pre>
 * BaseMechanism mechanism = new BaseMechanism(
 *     "Intake", 
 *     new MotorInterface[] {motor1, motor2},
 *     new SensorInterface[] {beamBreak},
 *     (motors, values) -> {
 *         for (int i = 0; i < motors.length; i++) {
 *             motors[i].setDuty(values[i]);
 *         }
 *     }
 * );
 * 
 * // Add trigger to stop when sensor detects game piece
 * mechanism.addStop(() -> beamBreak.get());
 * </pre>
 * 
 * @param <T> The concrete mechanism type (for method chaining)
 */
public class BaseMechanism<T extends BaseMechanism<T>> extends SubsystemBase{
    public static class MechanismAction {
        private String name;
        private Supplier<double[]> valuesChanger;
        private List<BiConsumer<MotorInterface[], double[]>> motorAndValuesInitializes;
        private List<Consumer<MotorInterface[]>> motorInitializes;
        private List<Runnable> runnableInitializes;
        private List<BiConsumer<MotorInterface[], double[]>> motorAndValuesExecutes;
        private List<Consumer<MotorInterface[]>> motorExecutes;
        private List<Runnable> runnableExecutes;
        private List<Supplier<Boolean>> finishes;
        private List<BiConsumer<MotorInterface[], double[]>> motorAndValuesEnds;
        private List<Consumer<MotorInterface[]>> motorEnds;
        private List<Runnable> runnableEnds;

        private boolean isCalibrateCommand;

        public MechanismAction(String name, Supplier<double[]> valuesChanger){
            
            if (valuesChanger == null) {
                throw new NullPointerException("Values supplier cannot be null");
            }
            this.name = name;
            this.valuesChanger = valuesChanger;
            this.motorAndValuesInitializes = new ArrayList<>();
            this.motorInitializes = new ArrayList<>();
            this.runnableInitializes = new ArrayList<>();
            this.motorAndValuesExecutes = new ArrayList<>();
            this.motorExecutes = new ArrayList<>();
            this.runnableExecutes = new ArrayList<>();
            this.finishes = new ArrayList<>();
            this.motorAndValuesEnds = new ArrayList<>();
            this.motorEnds = new ArrayList<>();
            this.runnableEnds = new ArrayList<>();
        }
        
        public MechanismAction(String name, double[] values){
            this(name, () -> values);
        }

        public String getName(){
            return name;
        }

        public MechanismAction withInitialize(BiConsumer<MotorInterface[], double[]> consumer){
            if (consumer == null) {
                throw new NullPointerException("Initialize consumer cannot be null");
            }
            motorAndValuesInitializes.add(consumer);
            return this;
        }

        public MechanismAction withInitialize(Consumer<MotorInterface[]> consumer){
            if (consumer == null) {
                throw new NullPointerException("Initialize consumer cannot be null");
            }
            motorInitializes.add(consumer);
            return this;
        }

        public MechanismAction withInitialize(Runnable consumer){
            if (consumer == null) {
                throw new NullPointerException("Initialize consumer cannot be null");
            }
            runnableInitializes.add(consumer);
            return this;
        }

        public MechanismAction withExecute(BiConsumer<MotorInterface[], double[]> consumer){
            if (consumer == null) {
                throw new NullPointerException("Execute consumer cannot be null");
            }
            motorAndValuesExecutes.add(consumer);
            return this;
        }

        public MechanismAction withExecute(Consumer<MotorInterface[]> consumer){
            if (consumer == null) {
                throw new NullPointerException("Execute consumer cannot be null");
            }
            motorExecutes.add(consumer);
            return this;
        }

        public MechanismAction withExecute(Runnable consumer){
            if (consumer == null) {
                throw new NullPointerException("Execute consumer cannot be null");
            }
            runnableExecutes.add(consumer);
            return this;
        }

        public MechanismAction withFinish(Supplier<Boolean> finish){
            if (finish == null) {
                throw new NullPointerException("Finish condition cannot be null");
            }
            finishes.add(finish);
            return this;
        }

        public MechanismAction withTime(double seconds) {
            Timer timer = new Timer();
            
            this.runnableInitializes.add(() -> {
                timer.restart();
            });
            
            this.finishes.add(() -> timer.hasElapsed(seconds));
            
            return this;
        }

        public MechanismAction withEnd(BiConsumer<MotorInterface[], double[]> consumer){
            if (consumer == null) {
                throw new NullPointerException("End consumer cannot be null");
            }
            motorAndValuesEnds.add(consumer);
            return this;
        }

        public MechanismAction withEnd(Consumer<MotorInterface[]> consumer){
            if (consumer == null) {
                throw new NullPointerException("End consumer cannot be null");
            }
            motorEnds.add(consumer);
            return this;
        }

        public MechanismAction withEnd(Runnable consumer){
            if (consumer == null) {
                throw new NullPointerException("End consumer cannot be null");
            }
            runnableEnds.add(consumer);
            return this;
        }

        public MechanismAction withValuesChanger(Supplier<double[]> valuesChanger){
            if (valuesChanger == null) {
                throw new NullPointerException("Values supplier cannot be null");
            }
            this.valuesChanger = valuesChanger;
            return this;
        }

        public MechanismAction setIsCalibrateCommand(boolean isCalibrateCommand){
            this.isCalibrateCommand = isCalibrateCommand;
            return this;
        }

        public Supplier<double[]> getValuesChanger(){
            return valuesChanger;
        }

        public double[] getValues(){
            return valuesChanger.get();
        }

        public List<BiConsumer<MotorInterface[], double[]>> getMotorAndValuesInitializes(){
            return motorAndValuesInitializes;
        }

        public List<Consumer<MotorInterface[]>> getMotorInitializes(){
            return motorInitializes;
        }

        public List<Runnable> getRunnableInitializes(){
            return runnableInitializes;
        }

        public List<BiConsumer<MotorInterface[], double[]>> getMotorAndValuesExecutes(){
            return motorAndValuesExecutes;
        }

        public List<Consumer<MotorInterface[]>> getMotorExecutes(){
            return motorExecutes;
        }

        public List<Runnable> getRunnableExecutes(){
            return runnableExecutes;
        }
        
        public List<Supplier<Boolean>> getFinishes(){
            return finishes;
        }

        public List<BiConsumer<MotorInterface[], double[]>> getMotorAndValuesEnd(){
            return motorAndValuesEnds;
        }

        public List<Consumer<MotorInterface[]>> getMotorEnds(){
            return motorEnds;
        }

        public List<Runnable> getRunnableEnds(){
            return runnableEnds;
        }

        public boolean getIsCalibrateCommand(){
            return isCalibrateCommand;
        }
    }

    public static class MotorLimits {
        private final double min;
        private final double max;

        public MotorLimits(double min, double max) {
            if (min > max) {
                throw new IllegalArgumentException("Min limit cannot be greater than max limit");
            }
            this.min = min;
            this.max = max;
        }

        public double clamp(double value) {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        public double getMin() {
            return min;
        }

        public double getMax() {
            return max;
        }
    }

    public static MechanismAction createAction(String name, Supplier<double[]> valuesChanger){
        return new MechanismAction(name, valuesChanger);
    }

    public static MechanismAction createAction(String name, double[] values){
        return new MechanismAction(name, values);
    }
    
    public MechanismAction createCalibrationAction(int motorIndex, double power, Supplier<Boolean> stopCondition, double resetVal, double upPower, double sec){
        if (!isValidMotorIndex(motorIndex)) {
            throw new IllegalArgumentException("Invalid motor index: " + motorIndex);
        }

        Timer timer = new Timer();
        double[] values = new double[motors.length];

        Supplier<double[]> powersChanger = () -> {
            values[motorIndex] = (timer.get() < sec) ? upPower : power;
            return values;
        };
        return new MechanismAction(name + " Calibration2", powersChanger)
            .setIsCalibrateCommand(true)
            .withInitialize(() -> setNeutralModeAll(true))
            .withInitialize(() -> timer.reset())
            .withInitialize(() -> timer.start())
            .withExecute((m, p) -> m[motorIndex].setDuty(p[motorIndex]))
            .withFinish(stopCondition)
            .withEnd(() -> {getMotor(motorIndex).setEncoderPosition(resetVal);})
            .withEnd(() -> {isCalibratedSupplier = () -> true;});
    }

    protected String name;
    protected MotorInterface[] motors;
    protected SensorInterface[] sensors;

    protected Supplier<Boolean> isCalibratedSupplier = () -> true;
    protected Supplier<Boolean> stopSupplier = () -> false;

    protected MotorLimits[] motorsLimits;
    protected BiConsumer<MotorInterface[], double[]> consumer = (m, v) -> {};
    protected Supplier<double[]> valuesChanger;
    protected Supplier<Double>[] valueModifiers;
    protected double[] lastCalculatedValues;

    protected HashMap<String, MechanismAction> actions = new HashMap<>();
    protected HashMap<String, Command> actionCommands = new HashMap<>();

    public BaseMechanism(String name) {
        this.name = name;
        SmartDashboard.putData(this);
    }

    @SuppressWarnings("unchecked")
    public T withMotors(MotorInterface ... motors){
        if (motors == null) {
            throw new NullPointerException("Motors array cannot be null");
        }
        if (motors.length == 0) {
            throw new IllegalArgumentException("Motors array cannot be empty");
        }
        for (int i = 0; i < motors.length; i++) {
            if (motors[i] == null) {
                throw new IllegalArgumentException("Motor at index " + i + " cannot be null");
            }
        }
        this.motors = motors;
        valueModifiers = new Supplier[motors.length];
        motorsLimits = new MotorLimits[motors.length];
        lastCalculatedValues = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            valueModifiers[i] = () -> 0.0;
        }
        motorsLimits = new MotorLimits[motors.length];
        valuesChanger = () -> new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            final int index = i;
            SmartDashboard.putData(getName() + "/" + getMotor(i).name() + "/set brake", 
                new InstantCommand(() -> setNeutralMode(index, true)).ignoringDisable(true));
        }
        for (int i = 0; i < motors.length; i++) {
            final int index = i;
            SmartDashboard.putData(getName() + "/" + getMotor(i).name() + "/set coast", 
                new InstantCommand(() -> setNeutralMode(index, false)).ignoringDisable(true));
        }
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withSensors(SensorInterface ... sensors){
        if (sensors == null) {
            throw new NullPointerException("Sensors array cannot be null");
        }
        for (int i = 0; i < sensors.length; i++) {
            if (sensors[i] == null) {
                throw new IllegalArgumentException("Sensor at index " + i + " cannot be null");
            }
        }
        this.sensors = sensors;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withConsumer(BiConsumer<MotorInterface[], double[]> consumer){
        if (consumer == null) {
            throw new NullPointerException("Consumer cannot be null");
        }
        this.consumer = consumer;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withValueChanger(Supplier<double[]> valuesChanger){
        if (valuesChanger == null) {
            throw new NullPointerException("Values changer cannot be null");
        }
        this.valuesChanger = valuesChanger;
        LogManager.addEntry(name + " values", valuesChanger)
            .withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();
        return (T) this;
    }

    /**
     * Adds a dynamic modifier (like arm gravity compensation) to a specific motor.
     */
    @SuppressWarnings("unchecked")
    public T withModifier(int index, Supplier<Double> modifier) {
        if (!isValidMotorIndex(index)) throw new IllegalArgumentException("Index out of bounds");
        this.valueModifiers[index] = modifier;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withMotorLimits(int motorIndex, double min, double max) {
        if (!isValidMotorIndex(motorIndex)) {
            throw new IllegalArgumentException("Invalid motor index: " + motorIndex);
        }
        motorsLimits[motorIndex] = new MotorLimits(min, max);
        return (T) this;
    }

    public T withMotorMinLimit(int motorIndex, double min) {
        return withMotorLimits(motorIndex, min, Double.POSITIVE_INFINITY);
    }

    public T withMotorMaxLimit(int motorIndex, double max) {
        return withMotorLimits(motorIndex, Double.NEGATIVE_INFINITY, max);
    }

    @SuppressWarnings("unchecked")
    public T withLookUpTable(LookUpTable lookUpTable, Supplier<Double> posSupplier){
        if (lookUpTable == null) {
            throw new NullPointerException("Lookup table cannot be null");
        }
        if (posSupplier == null) {
            throw new NullPointerException("Position supplier cannot be null");
        }
        this.valuesChanger = () -> lookUpTable.get(posSupplier.get());
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withStop(Supplier<Boolean> stopSupplier){
        this.stopSupplier = stopSupplier;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withAction(MechanismAction action){
        if (!action.getIsCalibrateCommand()) {
            action.withExecute(consumer);
        }
        actions.put(action.getName(), action);
        actionCommands.put(action.getName(), actionCommand(action.getName()));
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withButton(Trigger button, String actionName){
        if (button == null) {
            throw new NullPointerException("Button trigger cannot be null");
        }
        button.onTrue(actionCommands.get(actionName));
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withDriveMotor(int motorIndex, Trigger trigger, Supplier<Double> joystick){
        if (trigger == null) {
            throw new NullPointerException("Trigger cannot be null");
        }
        if (joystick == null) {
            throw new NullPointerException("Joystick supplier cannot be null");
        }
        if (!isValidMotorIndex(motorIndex)) {
            throw new IllegalArgumentException("Invalid motor index: " + motorIndex);
        }
        trigger.onTrue(new Command() {
            { addRequirements(BaseMechanism.this); }
            @Override
            public void execute() {
                getMotor(motorIndex).setDuty(joystick.get());
            }
        });
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withDefaultCommand(){
        if (motors == null) {
            throw new IllegalStateException("Motors must be configured before setting default command");
        }
        if (consumer == null) {
            throw new IllegalStateException("Consumer must be configured before setting default command");
        }
        this.setDefaultCommand(actionCommand(
            new MechanismAction(name + "DefaultCommand", valuesChanger)
        ));
        return (T) this;
    }

    public String getName(){
        return name;
    }

    public Command actionCommand(String actionName){
        return actionCommand(actions.get(actionName));
    }

    public Command actionCommand(MechanismAction action){
        if (action == null) {
            throw new NullPointerException("Action cannot be null");
        }
        if (motors == null) {
            throw new IllegalStateException("Motors must be configured before creating action command");
        }
        if (consumer == null) {
            throw new IllegalStateException("Consumer must be configured before creating action command");
        }
        Command command = new Command() {
            {
                addRequirements(BaseMechanism.this);
            }
            @Override
            public void initialize() {
                double[] currentValues = process(action.getValues());
                for (BiConsumer<MotorInterface[], double[]> motorAndValuesInitialize : action.getMotorAndValuesInitializes()){
                    motorAndValuesInitialize.accept(motors, currentValues);
                }
                for (Consumer<MotorInterface[]> motorInitialize : action.getMotorInitializes()){
                    motorInitialize.accept(motors);
                }
                for (Runnable motorInitialize : action.getRunnableInitializes()){
                    motorInitialize.run();
                }
            }

            @Override
            public void execute() {
                if (stopSupplier.get() || (!isCalibratedSupplier.get() && !action.getIsCalibrateCommand())) {
                    stopAll();
                    return; 
                }
                double[] currentValues = process(action.getValues());
                for (BiConsumer<MotorInterface[], double[]> motorAndValuesExecute : action.getMotorAndValuesExecutes()){
                    motorAndValuesExecute.accept(motors, currentValues);
                }
                for (Consumer<MotorInterface[]> motorExecute : action.getMotorExecutes()){
                    motorExecute.accept(motors);
                }
                for (Runnable motorExecute : action.getRunnableExecutes()){
                    motorExecute.run();
                }
            }

            @Override
            public boolean isFinished() {
                for (Supplier<Boolean> finish : action.getFinishes()){
                    if (finish.get()){
                        return true;
                    }
                }
                return false;
            }

            @Override
            public void end(boolean interrupted) {
                stopAll();
                double[] currentValues = process(action.getValues());
                for (BiConsumer<MotorInterface[], double[]> motorAndValuesEnd : action.getMotorAndValuesEnd()){
                    motorAndValuesEnd.accept(motors, currentValues);
                }
                for (Consumer<MotorInterface[]> motorEnd : action.getMotorEnds()){
                    motorEnd.accept(motors);
                }
                for (Runnable motorEnd : action.getRunnableEnds()){
                    motorEnd.run();
                }
            }
        }.withName(action.getName());
        SmartDashboard.putData(getName() + "/" + action.getName(), command);
        return command;
    }

    /**
     * Get the values that will be used by the mechanism consumer.
     * 
     * <p>These values are typically motor powers, positions, angles, or velocities.</p>
     */
    public double[] getValues(){
        return valuesChanger.get();
    }

    /**
     * Gets the limits for a specific motor.
     * 
     * @param motorIndex Index of motor
     * @return MotorLimits object or null if no limits set
     */
    public MotorLimits getMotorLimits(int motorIndex) {
        if (isValidMotorIndex(motorIndex)) {
            return motorsLimits[motorIndex];
        }
        return null;
    }

    /**
     * Processes base values through modifiers and limits.
     * @return A fresh array with processed values.
     */
    protected double[] process(double[] base) {
        double[] processed = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            double val = (base != null && i < base.length) ? base[i] : 0;
            val += valueModifiers[i].get();
            if (motorsLimits[i] != null) {
                val = motorsLimits[i].clamp(val);
            }
            processed[i] = val;
        }
        lastCalculatedValues = processed;
        return processed;
    }

    /**
     * Sets a calibration check function.
     * 
     * <p>Mechanism will only run if calibration supplier returns true.
     * Useful for mechanisms that need zeroing before use.</p>
     * 
     * @param isCalibratedSupplier Supplier that returns calibration status
     * @return this mechanism for chaining
     */
    @SuppressWarnings("unchecked")
    public T withCalibrationValue(Supplier<Boolean> isCalibratedSupplier){
        if (isCalibratedSupplier == null) {
            throw new NullPointerException("Calibration supplier cannot be null");
        }
        this.isCalibratedSupplier = isCalibratedSupplier;
        LogManager.addEntry(name + "/is calibrated", isCalibratedSupplier);
        return (T) this;
    }

    /**
     * Stops all motors immediately.
     */
    public void stopAll(){
        if (motors == null) return;
        for (MotorInterface motor : motors){
            motor.setDuty(0);
        }
    }

    /**
     * Stops a specific motor by index.
     * 
     * @param motorIndex Index of motor to stop
     */
    public void stop(int motorIndex){
        if (isValidMotorIndex(motorIndex)){
            motors[motorIndex].setDuty(0);
        }
    }

    /**
     * Sets power for all motors.
     * 
     * @param power Duty cycle (-1.0 to 1.0)
     */
    public void setPowerAll(double power) {
        if (motors == null) return;
        for (MotorInterface motor : motors){
            motor.setDuty(power);
        }
    }

    /**
     * Sets power for a specific motor.
     * 
     * @param motorIndex Index of motor
     * @param power Duty cycle (-1.0 to 1.0)
     */
    public void setPower(int motorIndex, double power){
        if (isValidMotorIndex(motorIndex)){
            motors[motorIndex].setDuty(power);
        }
    }

    /**
     * Sets neutral mode for all motors.
     * 
     * @param isBrake true for brake, false for coast
     */
    public void setNeutralModeAll(boolean isBrake) {
        if (motors == null) return;
        for (MotorInterface motor : motors) {
            if (motor != null) motor.setNeutralMode(isBrake);
        }
    }

    public void setNeutralMode(int motorIndex, boolean isBrake){
        if (isValidMotorIndex(motorIndex)){
            motors[motorIndex].setNeutralMode(isBrake);
        }
    }

    /**
     * Checks electronics for all motors and sensors.
     * 
     * <p>Logs any faults to console and telemetry.</p>
     */
    public void checkElectronicsAll() {
        if (motors == null) return;
        for (MotorInterface motor : motors) {
            if (motor != null) motor.checkElectronics();
        }
        if (sensors == null) return;
        for (SensorInterface sensor : sensors) {
            if (sensor != null) sensor.checkElectronics();
        }
    }

    public void checkElectronicsMotor(int motorIndex){
        if (isValidMotorIndex(motorIndex)){
            motors[motorIndex].checkElectronics();
        }
    }

    public void checkElectronicsSensor(int sensorIndex){
        if (isValidSensorIndex(sensorIndex)){
            sensors[sensorIndex].checkElectronics();
        }
    }

    /**
     * Gets a motor by index.
     * 
     * @param index Motor index
     * @return The motor interface
     * @throws IllegalArgumentException if index is invalid
     */
    public MotorInterface getMotor(int index) {
        if (isValidMotorIndex(index)) return motors[index];
        throw new IllegalArgumentException("Invalid motor index " + index);
    }

    /**
     * Gets a sensor by index.
     * 
     * @param index Sensor index
     * @return The sensor interface
     * @throws IllegalArgumentException if index is invalid
     */
    public SensorInterface getSensor(int index) {
        if (isValidSensorIndex(index)) return sensors[index];
        throw new IllegalArgumentException("Invalid sensor index " + index);
    }

    protected boolean isValidMotorIndex(int index) {
        return (motors != null) && (index >= 0 && index < motors.length);
    }

    protected boolean isValidSensorIndex(int index) {
        return (sensors != null) && (index >= 0 && index < sensors.length);
    }

    
    @Override
    public void periodic(){
        if (stopSupplier.get()){
            stopAll();
        }
    }
}
