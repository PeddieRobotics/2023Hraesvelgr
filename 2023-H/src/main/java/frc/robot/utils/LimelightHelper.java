package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * From:
 * https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java
 */
class LimelightTarget_Retro {

    @JsonProperty("t6c_ts")
    double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    double[] targetPose_RobotSpace;

    @JsonProperty("ta")
    double ta;

    @JsonProperty("tx")
    double tx;

    @JsonProperty("txp")
    double tx_pixels;

    @JsonProperty("ty")
    double ty;

    @JsonProperty("typ")
    double ty_pixels;

    @JsonProperty("ts")
    double ts;

    LimelightTarget_Retro() {
        cameraPose_TargetSpace = new double[6];
        robotPose_FieldSpace = new double[6];
        robotPose_TargetSpace = new double[6];
        targetPose_CameraSpace = new double[6];
        targetPose_RobotSpace = new double[6];
    }

}

class LimelightTarget_Fiducial {

    @JsonProperty("fid")
    double fiducialID;

    @JsonProperty("fam")
    String fiducialFamily;

    @JsonProperty("t6c_ts")
    double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    double[] targetPose_RobotSpace;

    @JsonProperty("ta")
    double ta;

    @JsonProperty("tx")
    double tx;

    @JsonProperty("txp")
    double tx_pixels;

    @JsonProperty("ty")
    double ty;

    @JsonProperty("typ")
    double ty_pixels;

    @JsonProperty("ts")
    double ts;

    LimelightTarget_Fiducial() {
        cameraPose_TargetSpace = new double[6];
        robotPose_FieldSpace = new double[6];
        robotPose_TargetSpace = new double[6];
        targetPose_CameraSpace = new double[6];
        targetPose_RobotSpace = new double[6];
    }
}

class LimelightTarget_Barcode {

}

class LimelightTarget_Classifier {

    @JsonProperty("class")
    String className;

    @JsonProperty("classID")
    double classID;

    @JsonProperty("conf")
    double confidence;

    @JsonProperty("zone")
    double zone;

    @JsonProperty("tx")
    double tx;

    @JsonProperty("txp")
    double tx_pixels;

    @JsonProperty("ty")
    double ty;

    @JsonProperty("typ")
    double ty_pixels;

    LimelightTarget_Classifier() {
    }
}

class LimelightTarget_Detector {

    @JsonProperty("class")
    String className;

    @JsonProperty("classID")
    double classID;

    @JsonProperty("conf")
    double confidence;

    @JsonProperty("ta")
    double ta;

    @JsonProperty("tx")
    double tx;

    @JsonProperty("txp")
    double tx_pixels;

    @JsonProperty("ty")
    double ty;

    @JsonProperty("typ")
    double ty_pixels;

    LimelightTarget_Detector() {
    }
}

class Results {

    @JsonProperty("pID")
    double pipelineID;

    @JsonProperty("tl")
    double latency_pipeline;

    @JsonProperty("tl_cap")
    double latency_capture;

    double latency_jsonParse;

    @JsonProperty("ts")
    double timestamp_LIMELIGHT_publish;

    @JsonProperty("ts_rio")
    double timestamp_RIOFPGA_capture;

    @JsonProperty("v")
    double valid;

    @JsonProperty("botpose")
    double[] botpose;

    @JsonProperty("botpose_wpired")
    double[] botpose_wpired;

    @JsonProperty("botpose_wpiblue")
    double[] botpose_wpiblue;

    @JsonProperty("Retro")
    LimelightTarget_Retro[] targets_Retro;

    @JsonProperty("Fiducial")
    LimelightTarget_Fiducial[] targets_Fiducials;

    @JsonProperty("Classifier")
    LimelightTarget_Classifier[] targets_Classifier;

    @JsonProperty("Detector")
    LimelightTarget_Detector[] targets_Detector;

    @JsonProperty("Barcode")
    LimelightTarget_Barcode[] targets_Barcode;

    Results() {
        botpose = new double[6];
        botpose_wpired = new double[6];
        botpose_wpiblue = new double[6];
        targets_Retro = new LimelightTarget_Retro[0];
        targets_Fiducials = new LimelightTarget_Fiducial[0];
        targets_Classifier = new LimelightTarget_Classifier[0];
        targets_Detector = new LimelightTarget_Detector[0];
        targets_Barcode = new LimelightTarget_Barcode[0];

    }
}

class LimelightResults {
    @JsonProperty("Results")
    Results targetingResults;

    LimelightResults() {
        targetingResults = new Results();
    }
}

public abstract class LimelightHelper {
    public static LimelightHelper createLimelightHelper(String name) {
        return new LimelightHelper() {
            @Override
            public String getName() {
                return sanitizeName(name);
            }
        };
    }

    private ObjectMapper mapper;

    /**
     * Print JSON Parse time to the console in milliseconds
     */
    boolean profileJSON = false;

    static String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "limelight";
        }
        return name;
    }

    /** returns the sanitized name */
    public abstract String getName(); 

    public NetworkTable getLimelightNTTable() {
        return NetworkTableInstance.getDefault().getTable(getName());
    }

    public NetworkTableEntry getLimelightNTTableEntry(String entryName) {
        return getLimelightNTTable().getEntry(entryName);
    }

    public double getLimelightNTDouble(String entryName) {
        return getLimelightNTTableEntry(entryName).getDouble(0.0);
    }

    public void setLimelightNTDouble(String entryName, double val) {
        getLimelightNTTableEntry(entryName).setDouble(val);
    }

    public void setLimelightNTDoubleArray(String entryName, double[] val) {
        getLimelightNTTableEntry(entryName).setDoubleArray(val);
    }

    public double[] getLimelightNTDoubleArray(String entryName) {
        return getLimelightNTTableEntry(entryName).getDoubleArray(new double[0]);
    }

    public String getLimelightNTString(String entryName) {
        return getLimelightNTTableEntry(entryName).getString("");
    }

    public URL getLimelightURLString(String request) {
        String urlString = "http://" + getName() + ".local:5807/" + request;
        URL url;
        try {
            url = new URL(urlString);
            return url;
        } catch (MalformedURLException e) {
            System.err.println("bad LL URL");
        }
        return null;
    }
    /////
    /////

    public double getTX() {
        return getLimelightNTDouble("tx");
    }

    public double getTY() {
        return getLimelightNTDouble("ty");
    }

    public double getTA() {
        return getLimelightNTDouble("ta");
    }

    public double getLatency_Pipeline() {
        return getLimelightNTDouble("tl");
    }

    public double getLatency_Capture() {
        return getLimelightNTDouble("tl_cap");
    }

    public double getCurrentPipelineIndex() {
        return getLimelightNTDouble("getpipe");
    }

    public String getJSONDump() {
        return getLimelightNTString("json");
    }

    public double[] getBotpose() {
        return getLimelightNTDoubleArray("botpose");
    }

    public double[] getBotpose_wpiRed() {
        return getLimelightNTDoubleArray("botpose_wpired");
    }

    public double[] getBotpose_wpiBlue() {
        return getLimelightNTDoubleArray("botpose_wpiblue");
    }

    public double[] getBotPose_TargetSpace() {
        return getLimelightNTDoubleArray("botpose_targetSpace");
    }

    public double[] getCameraPose_TargetSpace() {
        return getLimelightNTDoubleArray("camerapose_targetspace");
    }

    public double[] getTargetPose_CameraSpace() {
        return getLimelightNTDoubleArray("targetpose_cameraspace");
    }

    public double[] getTargetPose_RobotSpace() {
        return getLimelightNTDoubleArray("targetpose_robotspace");
    }

    public double[] getTargetColor() {
        return getLimelightNTDoubleArray("tc");
    }

    public double getFiducialID() {
        return getLimelightNTDouble("tid");
    }

    public double getNeuralClassID() {
        return getLimelightNTDouble("tclass");
    }

    /////
    /////

    public void setPipelineIndex(int pipelineIndex) {
        setLimelightNTDouble("pipeline", pipelineIndex);
    }

    /**
     * The LEDs will be controlled by Limelight pipeline settings, and not by robot
     * code.
     */
    public void setLEDMode_PipelineControl() {
        setLimelightNTDouble("ledMode", 0);
    }

    public void setLEDMode_ForceOff() {
        setLimelightNTDouble("ledMode", 1);
    }

    public void setLEDMode_ForceBlink() {
        setLimelightNTDouble("ledMode", 2);
    }

    public void setLEDMode_ForceOn() {
        setLimelightNTDouble("ledMode", 3);
    }

    public void setStreamMode_Standard() {
        setLimelightNTDouble("stream", 0);
    }

    public void setStreamMode_PiPMain() {
        setLimelightNTDouble("stream", 1);
    }

    public void setStreamMode_PiPSecondary() {
        setLimelightNTDouble("stream", 2);
    }

    /**
     * Sets the crop window. The crop window in the UI must be completely open for
     * dynamic cropping to work.
     */
    public void setCropWindow(double cropXMin, double cropXMax, double cropYMin,
            double cropYMax) {
        double[] entries = new double[4];
        entries[0] = cropXMin;
        entries[1] = cropXMax;
        entries[2] = cropYMin;
        entries[3] = cropYMax;
        setLimelightNTDoubleArray("crop", entries);
    }

    /////
    /////

    public void setPythonScriptData(double[] outgoingPythonData) {
        setLimelightNTDoubleArray("llrobot", outgoingPythonData);
    }

    public double[] getPythonScriptData() {
        return getLimelightNTDoubleArray("llpython");
    }

    /////
    /////

    /**
     * Asynchronously take snapshot.
     */
    public CompletableFuture<Boolean> takeSnapshot(String snapshotName) {
        return CompletableFuture.supplyAsync(() -> {
            return SYNCH_TAKESNAPSHOT(snapshotName);
        });
    }

    private boolean SYNCH_TAKESNAPSHOT(String snapshotName) {
        URL url = getLimelightURLString("capturesnapshot");
        try {
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            if (snapshotName != null && snapshotName != "") {
                connection.setRequestProperty("snapname", snapshotName);
            }

            int responseCode = connection.getResponseCode();
            if (responseCode == 200) {
                return true;
            } else {
                System.err.println("Bad LL Request");
            }
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
        return false;
    }

    /**
     * Parses Limelight's JSON results dump into a LimelightResults Object
     */
    public LimelightResults getLatestResults() {

        long start = System.nanoTime();
        LimelightResults results = new LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            results = mapper.readValue(getJSONDump(), LimelightResults.class);
        } catch (JsonProcessingException e) {
            System.err.println("lljson error: " + e.getMessage());
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.targetingResults.latency_jsonParse = millis;
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", millis);
        }

        return results;
    }
}