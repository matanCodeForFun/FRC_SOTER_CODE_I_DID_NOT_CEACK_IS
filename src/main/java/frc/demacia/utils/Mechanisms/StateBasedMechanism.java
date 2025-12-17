package frc.demacia.utils.Mechanisms;

import java.util.HashMap;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.Motors.MotorInterface;

/**
 * State machine-based mechanism controller.
 * 
 * <p>Extends BaseMechanism with enum-based state control and automatic state transitions.</p>
 * 
 * <p><b>Features:</b></p>
 * <ul>
 *   <li>Enum-based state definition</li>
 *   <li>Conditional state transitions</li>
 *   <li>Dashboard state selection</li>
 *   <li>Manual test mode</li>
 * </ul>
 * 
 * <p><b>Example Usage:</b></p>
 * <pre>
 * public enum IntakeStates implements Intake.IntakeState {
 *     IDLE(new double[]{0, 0}),
 *     INTAKING(new double[]{0.8, 0.8}),
 *     OUTTAKING(new double[]{-0.5, -0.5});
 *     
 *     private double[] values;
 *     IntakeStates(double[] values) { this.values = values; }
 *     public double[] getValues() { return values; }
 * }
 * 
 * Intake intake = new Intake("Intake", motors, sensors, IntakeStates.class)
 *     .withStartingOption(IntakeStates.IDLE)
 *     .addTrigger(() -> controller.getAButton(), IntakeStates.INTAKING)
 *     .addTrigger(() -> beamBreak.get(), IntakeStates.IDLE, IntakeStates.INTAKING);
 * </pre>
 * 
 * @param <T> The concrete mechanism type
 */
public class StateBasedMechanism<T extends StateBasedMechanism<T>> extends BaseMechanism<T> {

    public static class State {
        private String name;
        private double[] values;

        public State(String name, double[] values){
            if (values == null) {
                throw new NullPointerException("Values cannot be null");
            }
            this.name = name;
            this.values = values;
        }

        public String getName(){
            return name;
        }

        public void setValues(double[] values){
            this.values = values;
        }

        public double[] getValues(){
            return values;
        }
    }

    public static State creatState(String name, double[] values){
        return new State(name, values);
    }

    public String name;

    SendableChooser<State> stateChooser = new SendableChooser<>();
    public State state;
    protected State testingState;
    protected State idleState;
    protected State idleState2;

    protected double[] testValues;

    protected HashMap<String, State> states = new HashMap<>();

    public StateBasedMechanism(String name){
        super(name);
        withValueChanger(() -> getState() != null ? getState().getValues() : new double[0]);
    }

    @Override
    @SuppressWarnings("unchecked")
    public T withMotors(MotorInterface ... motors){
        super.withMotors(motors);
        testValues = new double[motors.length];
        double[] idleValues = new double[motors.length];
        testingState = new State("TESTING", testValues);
        idleState  = new State("IDLE", idleValues);
        idleState2 = new State("IDLE2", idleValues);
        stateChooser.addOption(testingState.getName(), testingState);
        stateChooser.addOption(idleState.getName(), idleState);
        stateChooser.addOption(idleState2.getName(), idleState2);
        stateChooser.onChange(state -> this.state = state);
        SmartDashboard.putData(getName() + "/State Chooser", stateChooser);
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withState(State state){
        stateChooser.addOption(state.getName(), state);
        states.put(state.getName(), state);
        return (T) this;
    }

    @Override
    public T withButton(Trigger button, String StateName){
        if (!states.containsKey(StateName)){
            return super.withButton(button, StateName);
        }
        return withButton(button, states.get(StateName));
    }

    @SuppressWarnings("unchecked")
    public T withButton(Trigger button, State state){
        if (button == null) {
            throw new NullPointerException("Button trigger cannot be null");
        }
        if (state == null) {
            throw new NullPointerException("state cannot be null");
        }
        button.onTrue(new InstantCommand(() -> setState(state)).ignoringDisable(true));
        return (T) this;
    }

    public T withStartingOption(String stateName){
        return withStartingOption(states.get(stateName));
    }

    /**
     * Sets the starting state for this mechanism.
     * 
     * @param state Initial state (must implement MechanismState)
     * @return this mechanism for chaining
     * @throws IllegalArgumentException if state doesn't implement MechanismState
     */
    @SuppressWarnings("unchecked")
    public T withStartingOption(State state){
        if (state == null) {
            throw new IllegalArgumentException("Starting state cannot be null");
        }

        stateChooser.setDefaultOption(state.getName(), state);
        this.state = state;
        return (T) this;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty(getName() + "/Test Values", () -> getTestValues(), testValues -> setTestValues(testValues));
    }

    /**
     * Gets the current state.
     * 
     * @return Current state enum
     */
    public void setState(State state) {
        this.state = state;
    }

    /**
     * Gets the current state.
     * 
     * @return Current state enum
     */
    public State getState() {
        return state;
    }

    public double[] getTestValues(){
        return testValues != null? testValues : new double[0];
    }

    public void setTestValues(double[] testValues){
        if (testingState == null) return;
        testingState.setValues(testValues);
        this.testValues = testValues;
    }
}