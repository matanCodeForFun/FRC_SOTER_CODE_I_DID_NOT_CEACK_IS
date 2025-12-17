// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Log;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.demacia.utils.Data;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;
import frc.robot.RobotContainer;

public class LogEntry<T> {

    private final LogManager logManager;

    DataLogEntry entry;
    Data<T> data;
    BiConsumer<T[], Long> consumer = null;
    String name;
    String metaData;
    Publisher ntPublisher;

    private boolean isFloat;
    private boolean isBoolean;
    private boolean isArray;

    public LogLevel logLevel;

    /*
        * Constructor with the suppliers and boolean if add to network table
    */
    LogEntry(String name, Data<T> data, LogLevel logLevel, String metaData) {

        logManager = LogManager.logManager;

        this.name = name;
        this.logLevel = logLevel;
        this.data = data;
        this.metaData = metaData;

        this.isFloat = data.isDouble();
        this.isBoolean = data.isBoolean();
        this.isArray = data.isArray();

        this.entry = createLogEntry(logManager.log, name, metaData);

        if (logLevel == LogLevel.LOG_AND_NT || (logLevel == LogLevel.LOG_AND_NT_NOT_IN_COMP && !RobotContainer.isComp())) {
            this.ntPublisher = createPublisher(logManager.table, name);
        } else {
            this.ntPublisher = null;
        }
    }

    public void addData(String name, Data<T> data, String metaData){
        this.name = this.name + " | " + name;
        this.metaData = this.metaData + " | " + metaData;
        if (this.data.getSignals() != null){
            this.data.expandWithSignals(data.getSignals());
        } else {
            this.data.expandWithSuppliers(data.getSuppliers());
        }
        this.isArray = this.data.isArray();
        if (ntPublisher != null) ntPublisher.close();
        
        if (entry != null) entry.finish();

        entry = createLogEntry(logManager.log, this.name, this.metaData);

        if (logLevel == LogLevel.LOG_AND_NT || (logLevel == LogLevel.LOG_AND_NT_NOT_IN_COMP && !RobotContainer.isComp())) {
            ntPublisher = createPublisher(logManager.table, this.name);
        } else {
            ntPublisher = null;
        }
    }

    /**
     * Retrieves the signals that would be removed by removeData with the given parameters.
     * Only works if the data contains signals (not suppliers).
     * 
     * @param nameIndex The index in the name (1-based, matching removeData)
     * @param dataIndex The starting index in the data array
     * @param count The number of signals to retrieve
     * @return Array of signals that would be removed, or null if signals don't exist or indices are invalid
     */
    @SuppressWarnings("unchecked")
    public StatusSignal<?>[] getSignals(int nameIndex, int dataIndex, int count) {
        if (data == null || data.getSignals() == null) {
            LogManager.log("getSignals: data has no signals", AlertType.kWarning);
            return null;
        }

        int actualNameIndex = nameIndex - 1;
        String[] parts = name.split(" \\| ");

        if (actualNameIndex >= parts.length || actualNameIndex < 0) {
            LogManager.log("getSignals: nameIndex out of range: " + nameIndex + " (parts length: " + parts.length + ")", AlertType.kWarning);
            return null;
        }

        try {
            StatusSignal<T>[] signals = data.getSignals();
            
            if (dataIndex < 0 || dataIndex >= signals.length) {
                LogManager.log("getSignals: dataIndex out of range: " + dataIndex + " (signals length: " + signals.length + ")", AlertType.kWarning);
                return null;
            }

            if (dataIndex + count > signals.length) {
                LogManager.log("getSignals: count exceeds available signals: dataIndex=" + dataIndex + ", count=" + count + ", signals.length=" + signals.length, AlertType.kWarning);
                return null;
            }

            StatusSignal<T>[] result = new StatusSignal[count];
            System.arraycopy(signals, dataIndex, result, 0, count);
            return result;
        } catch (Exception e) {
            LogManager.log("getSignals: failed to retrieve signals: " + e.getMessage(), AlertType.kError);
            return null;
        }
    }

    /**
     * Retrieves the suppliers that would be removed by removeData with the given parameters.
     * Only works if the data contains suppliers (not signals).
     * 
     * @param nameIndex The index in the name (1-based, matching removeData)
     * @param dataIndex The starting index in the data array
     * @param count The number of suppliers to retrieve
     * @return Array of suppliers that would be removed, or null if suppliers don't exist or indices are invalid
     */
    @SuppressWarnings("unchecked")
    public Supplier<T>[] getSuppliers(int nameIndex, int dataIndex, int count) {
        if (data == null || data.getSuppliers() == null) {
            LogManager.log("getSuppliers: data has no suppliers", AlertType.kWarning);
            return null;
        }

        int actualNameIndex = nameIndex - 1;
        String[] parts = name.split(" \\| ");

        if (actualNameIndex >= parts.length || actualNameIndex < 0) {
            LogManager.log("getSuppliers: nameIndex out of range: " + nameIndex + " (parts length: " + parts.length + ")", AlertType.kWarning);
            return null;
        }

        try {
            Supplier<T>[] suppliers = data.getSuppliers();
            
            if (dataIndex < 0 || dataIndex >= suppliers.length) {
                LogManager.log("getSuppliers: dataIndex out of range: " + dataIndex + " (suppliers length: " + suppliers.length + ")", AlertType.kWarning);
                return null;
            }

            if (dataIndex + count > suppliers.length) {
                LogManager.log("getSuppliers: count exceeds available suppliers: dataIndex=" + dataIndex + ", count=" + count + ", suppliers.length=" + suppliers.length, AlertType.kWarning);
                return null;
            }

            Supplier<T>[] result = new Supplier[count];
            System.arraycopy(suppliers, dataIndex, result, 0, count);
            return result;
        } catch (Exception e) {
            LogManager.log("getSuppliers: failed to retrieve suppliers: " + e.getMessage(), AlertType.kError);
            return null;
        }
    }

    public void removeData(int nameIndex, int dataIndex, int count) {
        int actualIndex = nameIndex - 1;

        String[] parts = name.split(" \\| ");

        if (actualIndex >= parts.length || actualIndex < 0) {
            LogManager.log("removeData: nameIndex out of range: " + nameIndex + " (parts length: " + parts.length + ")", AlertType.kWarning);
            return;
        }
        StringBuilder newName = new StringBuilder();
        for (int i = 0; i < parts.length; i++) {
            if (i != actualIndex) {
                if (newName.length() > 0) newName.append(" | ");
                newName.append(parts[i]);
            }
        }

        name = newName.toString();

        if (data != null) {
            try {
                if (data.getSignals() != null) {
                    data.removeSignalRange(dataIndex, count);
                } else if (data.getSuppliers() != null) {
                    data.removeSupplierRange(dataIndex, count);
                }
            } catch (Exception e) {
                LogManager.log("removeData: failed to remove from data: " + e.getMessage(), AlertType.kError);
            }
        }

        if (name.isEmpty()) {
            if (ntPublisher != null) {
                ntPublisher.close();
                ntPublisher = null;
            }
            entry = null;
            data = null;
            return;
        }

        if (ntPublisher != null) ntPublisher.close();

        entry = createLogEntry(logManager.log, name, metaData);
        if (logLevel == LogLevel.LOG_AND_NT || (logLevel == LogLevel.LOG_AND_NT_NOT_IN_COMP && !RobotContainer.isComp())) {
            ntPublisher = createPublisher(logManager.table, name);
        } else {
            ntPublisher = null;
        }
    }

    void log() {
        data.refresh();
        if (!data.hasChanged()) {
            return;
        }

        long time = data.getTime();

        appendEntry(time);

        if (ntPublisher != null) {
            publishToNetworkTable();
        }
        
        if (consumer != null) {
            consumer.accept(data.getValueArray(), time);
        } 
    }

    public void setConsumer(BiConsumer<T[], Long> consumer) {
        this.consumer = consumer;
    }

    public void removeInComp() {
        if (logLevel == LogLevel.LOG_AND_NT_NOT_IN_COMP && ntPublisher != null) {
            ntPublisher.close();
        }
    }

    private DataLogEntry createLogEntry(DataLog log, String name, String metaData) {
        if (isArray) {
            if (isFloat){
                return new FloatArrayLogEntry(log, name, metaData);
            } else if (isBoolean){
                return new BooleanArrayLogEntry(log, name, metaData);
            } else{
                return new StringArrayLogEntry(log, name, metaData);
            }
        } else {
            if (isFloat){
                return new FloatLogEntry(log, name, metaData);
            } else if (isBoolean){
                return new BooleanLogEntry(log, name, metaData);
            } else{
                return new StringLogEntry(log, name, metaData);
            }
        }
    }

    private Publisher createPublisher(NetworkTable table, String name) {
        if (isArray) {
            if (isFloat){
                return table.getFloatArrayTopic(name).publish();
            } else if (isBoolean){
                return table.getBooleanArrayTopic(name).publish();
            } else{
                return table.getStringArrayTopic(name).publish();
            }
        } else {
            if (isFloat){
                return table.getFloatTopic(name).publish();
            } else if (isBoolean){
                return table.getBooleanTopic(name).publish();
            } else{
                return table.getStringTopic(name).publish();
            }
        }
    }

    private void appendEntry(long time) {
        if (isArray) {
            if (isFloat){
                ((FloatArrayLogEntry) entry).append(data.getFloatArray(), time);
            } else if (isBoolean){
                ((BooleanArrayLogEntry) entry).append(data.getBooleanArray(), time);
            } else{
                ((StringArrayLogEntry) entry).append(data.getStringArray(), time);
            }
        } else {
            if (isFloat){
                ((FloatLogEntry) entry).append(data.getFloat(), time);
            } else if (isBoolean){
                ((BooleanLogEntry) entry).append(data.getBoolean(), time);
            } else{
                ((StringLogEntry) entry).append(data.getString(), time);
            }
        }
    }

    private void publishToNetworkTable() {
        if (isArray) {
            if (isFloat){
                ((FloatArrayPublisher) ntPublisher).set(data.getFloatArray());
            } else if (isBoolean){
                ((BooleanArrayPublisher) ntPublisher).set(data.getBooleanArray());
            } else{
                ((StringArrayPublisher) ntPublisher).set(data.getStringArray());
            }
        } else {
            if (isFloat){
                ((FloatPublisher) ntPublisher).set(data.getFloat());
            } else if (isBoolean){
                ((BooleanPublisher) ntPublisher).set(data.getBoolean());
            } else{
                ((StringPublisher) ntPublisher).set(data.getString());
            }
        }
    }
}