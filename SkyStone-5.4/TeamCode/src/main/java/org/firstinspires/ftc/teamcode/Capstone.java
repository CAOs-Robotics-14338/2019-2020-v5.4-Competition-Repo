package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Capstone {
    // Setup variables and devices
    Servo cap;

    private double extended = 0;
    private double retracted = 1;


    // Constructor for capstone servo
    public Capstone(Servo capstone){
        cap = capstone;

    }

    // Method for extending the capstone by pressing one button and retracting it by pressing a different button
    public void CapControl(boolean EnabledState, boolean DisabledState){

        if(EnabledState){
            cap.setPosition(extended);
        }

        else if(DisabledState){
            cap.setPosition(retracted);
        }


    }

}
