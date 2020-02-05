
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Systems {
    //Set Up Variables and Hardware Devices
    private DcMotor IntakeRightMotor, IntakeLeftMotor;
    private Servo IntakePulley, Claw;
    double active = 1;//0
    double inactive = 0; //1
//Set Up Touch Sensor
    DigitalChannel touch;

//Constructor for using Subsystem for Automated Tasks with the Touch Sensor and Claw and Pulley
    public Intake_Systems(DcMotor rightIntake, DcMotor leftIntake, Servo pulley, DigitalChannel ttouch, Servo cclaw) {
        IntakeLeftMotor = leftIntake;
        IntakeRightMotor = rightIntake;
        IntakePulley = pulley;
        touch = ttouch;
        Claw = cclaw;
    }
    //Constructor for using Subsystem for Automated Tasks with the Touch Sensor and Pulley
    public Intake_Systems(DcMotor rightIntake, DcMotor leftIntake, Servo pulley, DigitalChannel ttouch) {
        IntakeLeftMotor = leftIntake;
        IntakeRightMotor = rightIntake;
        IntakePulley = pulley;
        touch = ttouch;
    }
    //Constructor for using Subsystem without Automated Tasks
    public Intake_Systems(DcMotor rightIntake, DcMotor leftIntake, Servo pulley) {
        IntakeLeftMotor = leftIntake;
        IntakeRightMotor = rightIntake;
        IntakePulley = pulley;

    }

    //Main intake method for Teleop
    // If either set of controlling buttons are pressed run the designated command.
    public void intakeTele(boolean collect1, boolean deploy1, boolean collect2, boolean deploy2){
        if(collect1 || collect2){
            IntakeLeftMotor.setPower(0.95);
            IntakeRightMotor.setPower(-0.95);
        }
        else if(deploy1 || deploy2){
            IntakeLeftMotor.setPower(-0.95);
            IntakeRightMotor.setPower(0.95);
        }
        else {
            IntakeLeftMotor.setPower(0);
            IntakeRightMotor.setPower(0);
        }

    }

    //Intake method without use of two different control sets
    public void intake(boolean collect1, boolean deploy1){
        if(collect1){

            IntakeLeftMotor.setPower(0.95);
            IntakeRightMotor.setPower(-0.95);
        }
        else if(deploy1){
            IntakeLeftMotor.setPower(-0.95);
            IntakeRightMotor.setPower(0.95);
        }
        else{
            IntakeLeftMotor.setPower(0);
            IntakeRightMotor.setPower(0);
        }

    }

    //Automoated Teleop Intake Control
    // When the intake is set to run, the claw goes up so it is out of the way and won't
   // be in the way of the block.
   public void inTel(boolean collect1, boolean deploy1){
        if(collect1){
            IntakePulley.setPosition(active);
            Claw.setPosition(0.5);
            IntakeLeftMotor.setPower(0.95);
            IntakeRightMotor.setPower(-0.95);
        }
        else if(deploy1){
            IntakeLeftMotor.setPower(-0.95);
            IntakeRightMotor.setPower(0.95);
        }
        else{
            IntakeLeftMotor.setPower(0);
            IntakeRightMotor.setPower(0);
        }

    }
//Gets the State of the Touch Sensor
    //When the stones are pulled in by the intake and it hits the touch sensor, the touch sensor will
    //return true
    public boolean getTouch(){ //returns true if it's touched
        if (touch.getState() == true) {
            return false;
        } else {
            return true;
        }
    }


   //Method for Controlling the Collection Arms for Automated Parts

    //When the designated button is pressed, it will retract the collection arms, so they are out of
    //the way for the scissor lift and raising the block.
    public void pullBackCollectionArms(boolean retract1){
        if(retract1 ){
            IntakePulley.setPosition(inactive);}

    }
    //When the designated button is pressed, it will release the collection arms, so they are able
    //to collect stones again.
    public void releaseCollectionArms(boolean release1){
        if(release1){
            IntakePulley.setPosition(active);}

    }
     // Method for stopping the Intake Motors
        public void stopCollection(){
            IntakeLeftMotor.setPower(0);
            IntakeRightMotor.setPower(0);
        }


}
