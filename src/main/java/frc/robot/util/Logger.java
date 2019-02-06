package frc.robot.util;

import java.io.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

 
public class Logger {
   
    private BufferedWriter writer;
    private boolean logging =true; 
    private final String loggerBoolean = "Logging";
    private static Logger instance;
    private String fileName ="theorylog";
    private final String SDFileName = "File Name: ";
    DriverStation ds;
    
    private int max = 0;
    
    private String path;
    
    public static Logger getInstance() {
        if(instance == null) {
            instance = new Logger();
        }
        return instance;
    }
 
    private Logger() {
        this.ds = DriverStation.getInstance();
        SmartDashboard.putBoolean(this.loggerBoolean, this.logging);
//        this.logging= SmartDashboard.getBoolean(this.loggerBoolean);
        
        SmartDashboard.putString(this.SDFileName, this.fileName);

        File f = new File("/home/lvuser/theorylogs");
        if(!f.exists()) {
        	f.mkdir();
        }
        
    	File[] files = new File("/home/lvuser/theorylogs").listFiles();
    	if(files != null) {
	        for(File file : files) {
	            if(file.isFile()) {
//	                System.out.println(file.getName());
	                try {
	                    int index = Integer.parseInt(file.getName().split("_")[0]);
	                    if(index > max) {
	                        max = index;
	                    }
	                } catch (Exception e){
	                    e.printStackTrace();
	                }
	            }
	        }
    	} else {
    		max = 0;
    	}
    }
	    
    public void openFile(String mode) {
    	if(this.wantToLog() || this.ds.isFMSAttached()){
	        try{
	            path = this.getPath();
	            this.writer = new BufferedWriter(new FileWriter(path));
	            this.writer.write("Start of: " + mode);
	            //this.writer.write("FPGATime, encLeft, encRight, velLeft, velRight, leftEncSetp, rightEncSetp, leftVelSetp, rightVelSetp, gyroSetp, yaw, angle, ");
	            //this.writer.write(String.format("%.3f", (double)RobotMap.cruiseVelocity));
	            this.writer.newLine();
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
    	}
    }
    
    private String getPath() {
    	this.fileName = SmartDashboard.getString(SDFileName, fileName);
        if(this.ds.isFMSAttached()) {
            return String.format("/home/lvuser/theorylogs/%d_%s_%d_log.csv", ++this.max, this.ds.getAlliance().name(), this.ds.getLocation());
        }else if(this.fileName != null){ 
        	return String.format("/home/lvuser/theorylogs/%d_%s.csv",++this.max,this.fileName);
        }else {
            return String.format("/home/lvuser/theorylogs/%d_log.csv", ++this.max);
        }
    }
   
    public void logd(String tag, String text){
    	try {
        	this.writer.write(tag + ": " + text);
            
            this.writer.newLine();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    public boolean wantToLog(){
    	//this.logging= SmartDashboard.getBoolean(this.loggerBoolean);
    	return this.logging;
    }
    
    
    
    public void close() {
    	if(this.wantToLog()){
	    	if(this.writer != null) {
	            try {
	                this.writer.close();
	            }
	            catch (IOException e) {
	                e.printStackTrace();
	            }
	    	}
    	}
    }
}