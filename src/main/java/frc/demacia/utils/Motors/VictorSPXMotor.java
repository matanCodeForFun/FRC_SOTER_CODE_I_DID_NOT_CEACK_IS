package frc.demacia.utils.Motors;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.Log.LogManager;

public class VictorSPXMotor  extends VictorSPX implements MotorInterface,Sendable {
    VictorSPXConfig config;//TODO
    String name;

    public VictorSPXMotor(VictorSPXConfig config) {
        super(config.id);
        this.config = config;
        name = config.name;
        configMotor();
        addLog();
        LogManager.log(name + " motor initialized");
        SmartDashboard.putData(name, this);
    }

    private void configMotor() {
        //TODO
    }

    private void addLog() {
        //TODO
    }

    public String name(){
        return "";//TODO
    }

    public void changeSlot(int slot){
        //TODO
    }

    public void setNeutralMode(boolean isBrake){
        //TODO
    }

    public void setDuty(double power){
        //TODO
    }

    public void setVoltage(double voltage){
        //TODO
    }

    public void setVelocity(double velocity, double feedForward){
        //TODO
    }

    public void setVelocity(double velocity){
        //TODO
    }

    public void setMotion(double position, double feedForward){
        //TODO
    }

    public void setMotion(double position){
        //TODO
    }

    public void setAngle(double angle, double feedForward){
        //TODO
    }

    public void setAngle(double angle){
        //TODO
    }

    public void setPositionVoltage(double position, double feedForward){
        //TODO
    }

    public void setPositionVoltage(double position){
        //TODO
    }

    public void setVelocityWithFeedForward(double velocity){
        //TODO
    }

    public void setMotionWithFeedForward(double velocity){
        //TODO
    }

    public String getCurrentControlMode(){
        return "";//TODO
    }

    public double getCurrentClosedLoopSP(){
        return 0;//TODO
    }

    public double getCurrentClosedLoopError(){
        return 0;//TODO
    }

    public double getCurrentPosition(){
        return 0;//TODO
    }

    public double getCurrentAngle(){
        return 0;//TODO
    }

    public double getCurrentVelocity(){
        return 0;//TODO
    }

    public double getCurrentAcceleration(){
        return 0;//TODO
    }

    public double getCurrentVoltage(){
        return 0;//TODO
    }

    public double getCurrentCurrent(){
        return 0;//TODO
    }

    public void setEncoderPosition(double position){
        //TODO
    }

    public void showConfigPIDFSlotCommand(int slot){
        //TODO
    }

    public void showConfigMotionVelocitiesCommand(){
        //TODO
    }

    public void initSendable(SendableBuilder builder){
        //TODO
    }
}
