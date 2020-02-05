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

    private Servo claw, wrist;
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
    public ArmCollection(Servo claW, Servo wrisT, CRServo expansioN, Servo pulley){
        claw = claW;
        wrist = wrisT;
        expansion = expansioN;
        wrist.setPosition(MID_POS); //Make sure that the wrist initializes in the Middle Position
        // so it is out of the way of any electronics.
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

//Wrist
    // Wrist Control for Teleop with various ways to set the position of the wrist
    public void wristControl(double xstick, boolean right_bumper, boolean left_bumper, boolean start) {
        //Using the x value of a joystick the wrist will increase by the INCREMENT value.
        // If the right bumper is pressed, set the position to the minimum position or far right
        //If the left bumper is pressed, set the position to the maximum position or far left
        //If the start button is pressed, set the position to the middle position.
        double wristRange = Range.clip( (-xstick), -1.0, 1.0);
        if (wristRange > 0){
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
            }
        }
        else {
            if (wristRange < 0){
                //position = -1.0;
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                }
            }
        }
        if (right_bumper){
            position = MIN_POS;

        }
        if (left_bumper){
            position = MAX_POS;
        }
        if (start){
            position = MID_POS;
        }
        wrist.setPosition(position);
    }

    //Potential Different Way of Controlling the wrist with a joystick x-value
    //If the wristRange is greater than 0, then the wrist will add increment value until it reaches
    //the MAX_POS
    //If the the wristRange is less than 0, the the wrist will subtract the increment value until
    //it reaches the MIN_POS
    public void wristControl(double xstick) {
        double wristRange = Range.clip( (-xstick), -1.0, 1.0);
        if (wristRange > 0){
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
            }
        }
        else {
            if (wristRange < 0){
                //position = -1.0;
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                }
            }
        }

        wrist.setPosition(position);
    }

    //reset the wrist and set the intake arms ready to intakes
    public void reset(boolean button){
        // set wrist to straight ahead (middle position) and up and collector engaged and ready to collect
        wrist.setPosition(MID_POS);
        IntakePulley.setPosition(active);
       // // driver needs to retract expansion and lower the scissor lift possible code for later competition


    }




}