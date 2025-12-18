// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Data;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;

/**
 * Centralized logging system for robot telemetry and diagnostics.
 * 
 * <p>Features:</p>
 * <ul>
 *   <li>Automatic data logging to file</li>
 *   <li>NetworkTables publishing for dashboard viewing</li>
 *   <li>Smart log entry grouping for performance</li>
 *   <li>Hot-reload compatible</li>
 *   <li>Competition-aware (can disable NT in comp)</li>
 * </ul>
 * 
 * <p><b>Usage:</b></p>
 * <pre>
 * // In RobotContainer constructor
 * new LogManager();
 * 
 * // In subsystems
 * LogManager.addEntry("DriveLeft/Voltage", () -> leftMotor.getCurrentVoltage())
 *     .withLogLevel(LogLevel.LOG_AND_NT)
 *     .build();
 * </pre>
 */
public class LogManager extends SubsystemBase {

  public static LogManager logManager;

  public static boolean isComp;

  DataLog log;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Log");

  private static ArrayList<ConsoleAlert> activeConsole;

  ArrayList<LogEntry<?>> individualLogEntries = new ArrayList<>();
  
  LogEntry<?>[] categoryLogEntries = new LogEntry<?>[16];

  private Map<String, String[]> nameSplitCache = new HashMap<>();
  private Map<String, Integer[]> entryLocationMap = new HashMap<>();

  public LogManager() {
    if (logManager != null) {
      return;
    }
    logManager = this;
    isComp = DriverStation.isFMSAttached();
    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    
    activeConsole = new ArrayList<>();
    log("log manager is ready");
  }

  /**
   * Ensures the LogManager singleton exists.
   * calling this allows usage without explicit instantiation in RobotContainer.
   */
  private static void initializeIfNeeded() {
    if (logManager == null) {
      new LogManager();
    }
  }

  /**
   * Creates a log entry builder for StatusSignal-based logging (CTRE Phoenix 6).
   * 
   * <p>StatusSignals provide timestamp synchronization and efficient data transfer.</p>
   * 
   * @param name Log entry name (use "/" for hierarchy, e.g., "Drive/Left/Current")
   * @param statusSignals One or more CTRE StatusSignal objects
   * @return Builder for configuring the log entry
   */
  @SuppressWarnings("unchecked")
  public static <T> LogEntryBuilder<T> addEntry(String name, StatusSignal<T>... statusSignals) {
    initializeIfNeeded();
    return new LogEntryBuilder<T>(name, statusSignals);
  }

  /**
   * Creates a log entry builder for Supplier-based logging (REV, generic data).
   * 
   * <p>Suppliers are called each cycle to get fresh data.</p>
   * 
   * @param name Log entry name (use "/" for hierarchy)
   * @param suppliers One or more Supplier functions
   * @return Builder for configuring the log entry
   */
  @SuppressWarnings("unchecked")
  public static <T> LogEntryBuilder<T> addEntry(String name, Supplier<T>... suppliers) {
    initializeIfNeeded();
    return new LogEntryBuilder<T>(name, suppliers);
  }

  /**
   * Removes NetworkTables publishers for entries marked LOG_ONLY_NOT_IN_COMP.
   * 
   * <p>Call this when entering competition mode to reduce network traffic
   * and improve loop timing. Data continues to be logged to file.</p>
   */
  public static void removeInComp() {
    initializeIfNeeded();
    for (int i = 0; i < logManager.individualLogEntries.size(); i++) {
      logManager.individualLogEntries.get(i).removeInComp();
      if (logManager.individualLogEntries.get(i).getLogLevel() == LogLevel.LOG_ONLY_NOT_IN_COMP) {
        logManager.individualLogEntries.remove(logManager.individualLogEntries.get(i));
        i--;
      }
    }

    for (int i = 0; i < 4; i++) {
      if (logManager.categoryLogEntries[i] != null && logManager.categoryLogEntries[i] != null) {
        logManager.categoryLogEntries[i].removeInComp();
        if (logManager.categoryLogEntries[i].getLogLevel() == LogLevel.LOG_ONLY_NOT_IN_COMP) {
          logManager.categoryLogEntries[i] = null;
          int index = i;
          logManager.entryLocationMap.entrySet().removeIf(entry -> entry.getValue()[0] == index);
        }
      }
    }
  }
  
  /**
   * Clears all log entries. Useful for testing or hot-reload scenarios.
   */
  public static void clearEntries() {
    initializeIfNeeded();
    if (logManager != null) {
      logManager.individualLogEntries.clear();
      for (int i = 0; i < logManager.categoryLogEntries.length; i++) {
        if (logManager.categoryLogEntries[i] != null) {
          logManager.categoryLogEntries[i] = null;
        }
      }
      logManager.entryLocationMap.clear();
      logManager.nameSplitCache.clear();
    }
  }
  
  /**
   * Gets the total number of active log entries.
   * 
   * @return Number of entries being logged
   */
  public static int getEntryCount() {
    initializeIfNeeded();
    if (logManager == null) return 0;

    int count = logManager.individualLogEntries.size();
    for (LogEntry<?> entry2 : logManager.categoryLogEntries) {
      if (entry2 != null) {
        count++;
      }
    }
    return count;
  }

  /**
   * Finds a log entry by name.
   * 
   * @param name The entry name to search for
   * @return The log entry, or null if not found
   */
  public static LogEntry<?> findEntry(String name) {
    initializeIfNeeded();
    if (logManager == null) return null;
    
    Integer[] location = logManager.entryLocationMap.get(name);
    if (location == null) return null;
    
    int categoryIndex = location[0];
    
    if (categoryIndex == -1) {
      int index = location[0];
      if (index >= 0 && index < logManager.individualLogEntries.size()) {
        return logManager.individualLogEntries.get(index);
      }
    } else {
      return logManager.categoryLogEntries[categoryIndex];
    }
    
    return null;
  }

  /**
   * Removes a log entry by name.
   * 
   * <p>Useful for hot-reload or dynamic logging scenarios.</p>
   * 
   * @param name The entry name to remove
   * @return true if entry was found and removed, false otherwise
   */
  public static boolean removeEntry(String name) {
    initializeIfNeeded();
      if (logManager == null) return false;
      
      Integer[] location = logManager.entryLocationMap.get(name);
      if (location == null) return false;
      
      int categoryIndex = location[0];
      
      if (categoryIndex == -1) {
          int index = location[1];
          if (index >= 0 && index < logManager.individualLogEntries.size()) {
              logManager.individualLogEntries.remove(index);
              logManager.entryLocationMap.remove(name);
              logManager.nameSplitCache.remove(name);
              
              logManager.updateIndicesAfterRemoval(index);
              return true;
          }
      } else {
          int subIndex = location[1];
          int dataIndex = location[2];
          int dataCount = location[3];
          
          logManager.entryLocationMap.remove(name);
          logManager.nameSplitCache.remove(name);
          
          LogEntry<?> categoryEntry = logManager.categoryLogEntries[categoryIndex];
          if (categoryEntry != null) {
            categoryEntry.removeData(subIndex, dataIndex, dataCount);
            
            logManager.updateSubIndicesAfterRemoval(categoryIndex, subIndex);
            logManager.updateDataIndicesAfterRemoval(categoryIndex, dataIndex, dataCount);
            
            if (categoryEntry.getData() == null || 
              categoryEntry.getName() == null || 
              categoryEntry.getName().trim().isEmpty()) {
              logManager.categoryLogEntries[categoryIndex] = null;
              final int catIndex = categoryIndex;
              logManager.entryLocationMap.entrySet().removeIf(
                  entry -> entry.getValue()[0] == catIndex
              );
            }
          }
          
          return true;
      }
      
      return false;
  }

  /**
   * Logs a message with specified alert level.
   * 
   * <p>Messages are written to:</p>
   * <ul>
   *   <li>Driver Station console</li>
   *   <li>Log file</li>
   *   <li>On-screen alerts (for warnings/errors)</li>
   * </ul>
   * 
   * @param message The message to log
   * @param alertType Severity level (kInfo, kWarning, kError)
   * @return ConsoleAlert object for additional control
   */
  public static ConsoleAlert log(Object message, AlertType alertType) {
    initializeIfNeeded();
    DataLogManager.log(String.valueOf(message));
    
    ConsoleAlert alert = new ConsoleAlert(String.valueOf(message), alertType);
    alert.set(true);
    if (activeConsole.size() > ConsoleConstants.CONSOLE_LIMIT) {
      activeConsole.get(0).close();
      activeConsole.remove(0);
    }
    activeConsole.add(alert);
    return alert;
  }

  /**
   * Logs an info-level message.
   * 
   * @param message The message to log
   * @return ConsoleAlert object
   */
  public static ConsoleAlert log(Object message) {
    initializeIfNeeded();
    return log(message, AlertType.kInfo);
  }

  @Override
  public void periodic() {
    Data.refreshAll();

    for (int i = activeConsole.size() - 1; i >= 0; i--) {
      ConsoleAlert alert = activeConsole.get(i);
      if (alert.isTimerOver()) {
          alert.set(false);
          activeConsole.remove(i);
      }
    }

    for (LogEntry<?> e : individualLogEntries) {
      e.log();
    }
    for (LogEntry<?> e : categoryLogEntries) {
      if (e != null){
        e.log();
      }
    }
  }

  public <T> LogEntry<T> add(String name, Data<T> data, LogLevel logLevel, String metaData, boolean isSeparated) {
    LogEntry<T> entry = null;

    int categoryIndex = getCategoryIndex(data, logLevel, isSeparated);

    if (categoryIndex == -1){
      entry = new LogEntry<T>(name, data, logLevel, metaData);
      individualLogEntries.add(entry);
      int index = individualLogEntries.size() - 1;
      entryLocationMap.put(name, new Integer[] {-1, index, null, null});
    } else{
      entry = addToEntryArray(categoryIndex, name, data, metaData);
    }

    return entry;
  }

  @SuppressWarnings("unchecked")
  private <T> LogEntry<T> addToEntryArray(int i, String name, Data<T> data, String metaData) {
    if (categoryLogEntries[i] != null && categoryLogEntries[i].getData() != null) {
      boolean groupIsSignal = categoryLogEntries[i].getData().getSignals() != null;
      boolean newIsSignal = data.getSignals() != null;

      if (groupIsSignal != newIsSignal) {
          LogManager.log("Log Type Mismatch in '" + name + "'. Creating separate entry.", AlertType.kWarning);
          return add(name, data, LogLevel.LOG_ONLY, metaData, false);
      }
    }
  
    int subIndex;
    int dataIndex = 0;
    
    if (categoryLogEntries[i] == null) {
        categoryLogEntries[i] = new LogEntry<>(name, data, getLogLevelFromIndex(i), metaData);
        subIndex = 1;
        dataIndex = 0;
    } else {
        String[] parts = getCachedSplit(categoryLogEntries[i].getName());
        subIndex = parts.length + 1;

        if (categoryLogEntries[i].getData().getSignals() != null) {
          dataIndex = categoryLogEntries[i].getData().getSignals().length;
        } else if (categoryLogEntries[i].getData().getSuppliers() != null) {
          dataIndex = categoryLogEntries[i].getData().getSuppliers().length;
        }

        try {
            ((LogEntry<T>) categoryLogEntries[i]).addData(name, data, metaData);
        } catch (Exception e) {
            LogManager.log("Error combining log entries: " + e.getMessage(), AlertType.kError);
        }
    }

    int dataLength = (data.getSignals() != null) ? data.getSignals().length : data.getSuppliers().length;
    entryLocationMap.put(name, new Integer[]{i, subIndex, dataIndex, dataLength});
    
    return (LogEntry<T>) categoryLogEntries[i];
  }

  private LogLevel getLogLevelFromIndex(int i) {
    if (i <= 3) return LogLevel.LOG_ONLY_NOT_IN_COMP;
    if (i <= 7) return LogLevel.LOG_ONLY;
    if (i <= 11) return LogLevel.LOG_AND_NT_NOT_IN_COMP;
    return LogLevel.LOG_AND_NT;
  }

  private String[] getCachedSplit(String name) {
    return nameSplitCache.computeIfAbsent(name, k -> k.split(" \\| "));
  }

  private int getCategoryIndex(Data<?> data, LogLevel logLevel, Boolean isSeperated) {
    boolean isSignal = data.getSignals() != null;
    boolean isSupplier = data.getSuppliers() != null;
    boolean isDouble = data.isDouble();
    boolean isBoolean = data.isBoolean();
    
    if (!(isDouble || isBoolean) || !(isSignal || isSupplier) || isSeperated) {
      return -1;
    }
    
    int baseIndex = (isSignal ? 0 : 2) + (isDouble ? 0 : 1);
    int levelOffset;
    switch (logLevel) {
      case LOG_ONLY_NOT_IN_COMP:
        levelOffset = 0;
        break;
      case LOG_ONLY:
        levelOffset = 4;
        break;
      case LOG_AND_NT_NOT_IN_COMP:
        levelOffset = 8;
        break;
      default:
        levelOffset = 12;
        break;
    }
    
    return baseIndex + levelOffset;
  }

  private void updateIndicesAfterRemoval(int removedIndex) {
      for (Map.Entry<String, Integer[]> entry : entryLocationMap.entrySet()) {
          Integer[] location = entry.getValue();
          if (location[0] == -1 && location[1] > removedIndex) {
              entry.setValue(new Integer[]{-1, location[1] - 1, location[2], location[3]});
          }
      }
  }

  private void updateSubIndicesAfterRemoval(int categoryIndex, int removedSubIndex) {
      for (Map.Entry<String, Integer[]> entry : entryLocationMap.entrySet()) {
        Integer[] location = entry.getValue();
          if (location[0] == categoryIndex && location[1] > removedSubIndex) {
              entry.setValue(new Integer[]{categoryIndex, location[1] - 1, location[2], location[3]});
          }
      }
  }

  private void updateDataIndicesAfterRemoval(int categoryIndex, int removedDataIndex, int removedCount) {
      for (Map.Entry<String, Integer[]> entry : entryLocationMap.entrySet()) {
          Integer[] location = entry.getValue();
          if (location[0] == categoryIndex && 
              location[2] != null &&
              location[2] >= removedDataIndex + removedCount) {
              entry.setValue(new Integer[] {location[0], location[1], location[2] - removedCount, location[3]});
          }
      }
  }

  /**
   * Retrieves the Suppliers associated with a specific log name.
   * @param name The name used when adding the entry
   * @return Array of Pairs containing (Name, Supplier), or null if not found/wrong type.
   */
  @SuppressWarnings("unchecked")
  public static <T> Pair<String, Supplier<T>>[] getSuppliers(String name) {
    initializeIfNeeded();
    Integer[] loc = logManager.entryLocationMap.get(name);
    if (loc == null) return null;

    LogEntry<?> entry = (loc[0] == -1) 
        ? logManager.individualLogEntries.get(loc[1]) 
        : logManager.categoryLogEntries[loc[0]];

    if (entry == null || entry.getData() == null) return null;

    Supplier<T>[] allSuppliers = (Supplier<T>[]) entry.getData().getSuppliers();
    if (allSuppliers == null) return null; 

    int start = (loc[0] == -1) ? 0 : loc[2];
    int len = (loc[0] == -1) ? allSuppliers.length : loc[3];

    String[] subNames = parseSubNames(name, len);

    Pair<String, Supplier<T>>[] result = new Pair[len];
    for (int i = 0; i < len; i++) {
        result[i] = new Pair<>(subNames[i], allSuppliers[start + i]);
    }
    return result;
  }

  /**
   * Retrieves the StatusSignals associated with a specific log name.
   * @param name The name used when adding the entry
   * @return Array of Pairs containing (Name, StatusSignal), or null if not found/wrong type.
   */
  @SuppressWarnings("unchecked")
  public static <T> Pair<String, StatusSignal<T>>[] getSignals(String name) {
    initializeIfNeeded();
    Integer[] loc = logManager.entryLocationMap.get(name);
    if (loc == null) return null;

    LogEntry<?> entry = (loc[0] == -1) 
        ? logManager.individualLogEntries.get(loc[1]) 
        : logManager.categoryLogEntries[loc[0]];

    if (entry == null || entry.getData() == null) return null;

    StatusSignal<T>[] allSignals = (StatusSignal<T>[]) entry.getData().getSignals();
    if (allSignals == null) return null; 

    int start = (loc[0] == -1) ? 0 : loc[2];
    int len = (loc[0] == -1) ? allSignals.length : loc[3];

    String[] subNames = parseSubNames(name, len);

    Pair<String, StatusSignal<T>>[] result = new Pair[len];
    for (int i = 0; i < len; i++) {
        result[i] = new Pair<>(subNames[i], allSignals[start + i]);
    }
    return result;
  }

  private static String[] parseSubNames(String name, int count) {
    String cleanName = name;
    if (name.contains(":")) {
        cleanName = name.substring(name.indexOf(":") + 1);
    }
    
    String[] parts = cleanName.split(",");
    
    for(int i = 0; i < parts.length; i++) {
        parts[i] = parts[i].trim();
    }

    if (parts.length == count) {
        return parts;
    }
    
    String[] fallback = new String[count];
    for(int i = 0; i < count; i++) fallback[i] = name;
    return fallback; 
  }
}