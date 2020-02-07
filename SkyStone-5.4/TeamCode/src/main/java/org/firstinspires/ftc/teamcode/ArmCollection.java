package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmCollection {
    // //Set Up Variables and Servo Devices

    private Servo claw;
    private CRServo expansion;

    //Set Up Intake pulley for the automated tasks
    private Servo IntakePulley;
    double active = 1;//0
    double inactive = 0; //1



    ///WRIST
//Variables for wrist controls
    static final double INCREMENT   = 0.001;                         // Amount to increase position by
    static final double MAX_POS     =  1.0;      //0.55                  // Maximum rotational position
    static final double MID_POS     = 0.45;       //0.35                  //Middle init position
    static final double MIN_POS     =  0.0;      //0.15                  // Minimum rotational position
    double position = MID_POS; // set the initial position to the middle position just in case




//Constructor
    // This allows for use of all hardware components including the pulley which is used for automation.
    public ArmCollection(Servo claW, CRServo expansioN, Servo pulley){
        claw = claW;
        expansion = expansioN;
        IntakePulley = pulley;


    }


//Expansion
        //Method for using the expansion with a joystick on the controller

        public  void expandControl(double ystick){ //expansion is a continous servo
            double expandRange = Range.clip( (-ystick), -1.0, 1.0);
            // If the range is greater than zero set the power to 1, in order to expand out to place
            // blocks on the foundation
            // If the range is less than zero set power to -1 in order to expand back in to store.
            if (expandRange > 0){
                expansion.setPower(1);
            }
            else {
                if (expandRange < 0){
                    expansion.setPower(-1);                }
                else
                    expansion.setPower(0);
            }

    }

    //Controlling the Scissor Lift using the DPAD on the controller or two other buttons
    //WITH AUTOMATION
    public  void expandControlDPAD(boolean up, boolean down){ //expansion is continous servo

        if (up){ //While extending out, set the collector arms to move out in order to avoid hitting
            //the scissor arm and claw against the intake wheels.
            IntakePulley.setPosition(inactive); //Places moves the intake pulley to move the
            // collection arms out of the way of the expansion
            expansion.setPower(1);
        }
        else {

            if (down){ //While contracting back in, set the collection arms as inactive to move
//            them out of the way of the moving expansion.
                IntakePulley.setPosition(inactive);//Places moves the intake pulley to move the
                // collection arms out of the way of the expansion
                expansion.setPower(-1);                }
            else
                expansion.setPower(0);
        }
    }



//Claw

    //Method that controls claw to grab the stone that was intaked or sitting out on the field.
    public void grab(boolean button){
        //if the button pressed, intake wheels move out of the way so they don't get in the way of
        //scissor lift.
        if (button){
            IntakePulley.setPosition(inactive); //using the intake pulley moves the intake arms to
            //the stowed position so they are out of the way
            claw.setPosition(0.0);          // preferred set 0.0
        }
    }

    //Method that controls the claw and opens the claw in order to "release" the stone.
    public void release(boolean button) {
        //if the button is pressed, set the intake wheels to active in order to be ready to collect
        //another stone.
        if (button) {
            IntakePulley.setPosition(active); //using the intake pulley moves the intake arms into
            //the way so the intake system is able to work again.
            claw.setPosition(0.50);        // preferred set 0.50
        }
    }

    public void clawAuto(boolean state) {

            if(state){
                IntakePulley.setPosition(inactive); //using the intake pulley moves the intake arms to
                //the stowed position so they are out of the way
                claw.setPosition(0.0);          // preferred set 0.0
            }
            else{
                IntakePulley.setPosition(active); //using the intake pulley moves the intake arms to
                //the stowed position so they are out of the way
                claw.setPosition(0.5);          // preferred set 0.0

            }

    }

}