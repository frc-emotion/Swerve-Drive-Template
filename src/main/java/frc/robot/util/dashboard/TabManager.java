package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * Organize subsystems by Shuffleboard Tabs and utilizeDebbger
 * <br>
 * </br>
 * WIP
 */
public class TabManager {
    private static TabManager instance;
    private static String[] tabNames = { "DRIVETRAIN", "SHOOTER", "INDEXER", "CLIMB", "VISION", "AUTON"};

    public enum SubsystemTab {
        DRIVETRAIN, SHOOTER, INDEXER,
        CLIMB, VISION, AUTON
    }

    public TabManager() {
        for (int i = 0; i < tabNames.length; i++) {
            Shuffleboard.getTab(tabNames[i]);
        }
    }

    public static synchronized TabManager getInstance() {
        if (instance == null) {
            instance = new TabManager();
        }
        return instance;
    }

    public ShuffleboardTab accessTab(SubsystemTab tabs) {
        switch (tabs) {
            case DRIVETRAIN:
                return Shuffleboard.getTab("DRIVETRAIN");
            case SHOOTER:
                return Shuffleboard.getTab("SHOOTER");
            case INDEXER:
                return Shuffleboard.getTab("INDEXER");
            case CLIMB:
                return Shuffleboard.getTab("CLIMB");
            case VISION:
                return Shuffleboard.getTab("VISION");
            case AUTON:
                return Shuffleboard.getTab("AUTON");
            default:
                return Shuffleboard.getTab("DRIVETRAIN");
        }
    }

    public NetworkTableEntry addWidget(ShuffleboardTab tab, BuiltInWidgets widgetType, String name, Object defaultValue,
            int[] position, int[] size) {
        return tab.add(name, defaultValue)
                .withPosition(position[0], position[1])
                .withSize(size[0], size[1])
                .withWidget(widgetType)
                .getEntry();
    }

    public ComplexWidget addFieldWidget(ShuffleboardTab tab, BuiltInWidgets widgetType, String name,
            Field2d defaultValue,
            int[] position, int[] size) {
        return tab.add(name, defaultValue)
                .withPosition(position[0], position[1])
                .withSize(size[0], size[1])
                .withWidget(widgetType);
    }

    /** Puts all NT Data into a wpilog */
    public void logDriveOdometry(boolean beginLog) {
        if (beginLog) {
            DataLogManager.start();
        } else {
            DataLog log = DataLogManager.getLog();
            System.out.println(log);
        }

    }

}
