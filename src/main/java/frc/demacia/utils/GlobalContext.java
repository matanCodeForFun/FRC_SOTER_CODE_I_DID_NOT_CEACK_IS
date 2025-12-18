package frc.demacia.utils;

public class GlobalContext{
    private static double cycleTime = 0.02;

    public static double getCycleTime() {
        return cycleTime;
    }

    public static void setCycleTime(double cycleTime) {
        GlobalContext.cycleTime = cycleTime;
    }
}
