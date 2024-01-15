// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;

/**
 * Start of the CustomTables class
 */
public class CustomTables {
    // The FMSInfo Table and its entries
	private NetworkTable FMSInfo;
	private NetworkTableEntry isRedAlliance;
    
    // The TagInfo Table and its entries
    private NetworkTable TagInfo;
    private NetworkTableEntry time;
    private NetworkTableEntry bestResult;
    private NetworkTableEntry targetValid;
    private NetworkTableEntry BestResultId;
    private NetworkTableEntry detectionTime;

    // The PieceData table and its entries
    private NetworkTable PieceData;
    private NetworkTableEntry width;
	private NetworkTableEntry centerX;
    private NetworkTableEntry numCones;
    private NetworkTableEntry numCubes;

    // Singleton for CustomTables to ensure only one NetworkTables server is created
    private static CustomTables instance = null;
    public static synchronized CustomTables getInstance() {
        if (instance == null) {
            instance = new CustomTables();
        }

        return instance;
    }

    /**
     * The constructor for the CustomTables class
     */
    private CustomTables() {
        // Gets the default instance
        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

        // Creates the FMSInfo table and its entries
		FMSInfo       = ntInst .getTable("FMSInfo");
		isRedAlliance = FMSInfo.getEntry("IsRedAlliance"); // boolean
    }

    /****************************************************************************************** 
    *
    *    GETS VALUES FROM FMSINFO
    * 
    ******************************************************************************************/
    /**
     * Gets our alliance color from the FMS.
     * 
     * @return isRed
     */
    public boolean getIsRedAlliance() {
        return isRedAlliance.getBoolean(false);
    }
}

// End of the CustomTables class
