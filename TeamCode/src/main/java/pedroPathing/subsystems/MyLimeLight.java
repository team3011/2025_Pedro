package pedroPathing.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MyLimeLight {
    private final Limelight3A limelight;
    private double xLoc;
    private double yLoc;
    private double angle;

    public MyLimeLight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void start(int input){
        limelight.pipelineSwitch(input);
        limelight.start();
    }

    public void stop(){
        limelight.stop();
    }

    public boolean update(){
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Getting numbers from Python
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                xLoc = pythonOutputs[1];
                yLoc = pythonOutputs[2];
                angle = pythonOutputs[3];
            }
            return true;
        }
        return false;
    }

    public double getxLoc(){
        return xLoc;
    }

    public double getyLoc(){
        return yLoc;
    }

    public double getAngle(){
        return angle;
    }




}
