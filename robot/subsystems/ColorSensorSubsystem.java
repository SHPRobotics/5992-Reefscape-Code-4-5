// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class ColorSensorSubsystem extends SubsystemBase{
    private final ColorSensorV3 colorSensor;
    private final double setDis;
    public String color = "No color";

    // Constructor
    public ColorSensorSubsystem(){
        colorSensor = new ColorSensorV3(Port.kOnboard);
        setDis = 40;
    }

    public void getColors(double red, double blue, double green, double distance){
//        String color = "No color";

        SmartDashboard.putNumber("Red Value :", red);
        SmartDashboard.putNumber("Blue Value :", blue);
        SmartDashboard.putNumber("Green Value :", green);
        SmartDashboard.putNumber("Proximity :", distance);
        SmartDashboard.putBoolean("Color Sensor Status :", colorSensor.isConnected());

       if (red > blue && green < (red + 200) && distance > setDis) {
            color = "Red";

        } else if (blue > red && green < (blue + 200) && distance > setDis){
            color = "Blue";
        } else if (green > red && green > blue && distance > setDis){
            color = "Green";
        }

        SmartDashboard.putString("COLOR DETECTED : ", color);
        
    }

    public ColorSensorV3 getSensor(){
        return colorSensor;
    }

    public String getColor(){
        /*getColors(getSensor().getRed(),
                            getSensor().getBlue(),
                            getSensor().getGreen(),
                            getSensor().getProximity());*/
        return color;
    }

    /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean isColorRed() {
    // Query some boolean state, such as a digital sensor.
    //if color is Red, returns true, else returns false
    System.out.println("In isColorRed(): " + getColor().equals("Red"));
    return getColor().equals("Red");

  }

}
