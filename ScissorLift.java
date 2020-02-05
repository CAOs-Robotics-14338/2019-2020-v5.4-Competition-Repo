package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ScissorLift {
    //Set Up Variables and Hardware Devices
    static final double MOTOR_TICK_COUNT = 1120; //1440
    private DcMotor ScissorLiftMotorLeft, ScissorLiftMotorRight;
    int initial = 560; //This is the amount that we need to raise the scissor lift to compensate for the foundation height
    int block = 1120; //This is the amount that the scissor lift must rise for 1 block
    int pos = 0;
    boolean pressed = false;
    int target = 0;

    //Set Up IntakePulley for automatic motion
    Servo IntakePulley;
    double active = 1;//0
    double inactive = 0; //1


    //Constructor 1 for using the Scissor Lift with Automated Actions
    public ScissorLift(DcMotor ScissorLiftL, DcMotor ScissorLiftR, Servo inPulley){
        ScissorLiftMotorLeft = ScissorLiftL;
        ScissorLiftMotorRight = ScissorLiftR;
        IntakePulley = inPulley;

    }
    //Constructor 2 for using the Scissor Lift without the Automated Actions
    public ScissorLift(DcMotor ScissorLiftL, DcMotor ScissorLiftR){
            ScissorLiftMotorLeft = ScissorLiftL;
            ScissorLiftMotorRight = ScissorLiftR;
        }

//Method in order to raise and lower the scissor lift using a joystick
    public  void LiftControl(double y2){
        double ScissorLiftPower = Range.clip( (-y2), -1.0, 1.0);
        ScissorLiftMotorLeft.setPower(ScissorLiftPower);
        ScissorLiftMotorRight.setPower(ScissorLiftPower);
        if (ScissorLiftPower > 0){ //When the scissor lift is lowered, the position of the intake
            // pulley will be set to its inactive position moving the collection arms out of the way
            // so the scissor lift does not hit the collection system.
            IntakePulley.setPosition(inactive);
        }

    }
    /*    public  void LiftControl(double y2){ //lift control for use without the automated parts
            double ScissorLiftPower = Range.clip( (-y2), -1.0, 1.0);
            ScissorLiftMotorLeft.setPower(ScissorLiftPower);
            ScissorLiftMotorRight.setPower(ScissorLiftPower);


        }*/

    //In the Process... Working for Total Automation of the Scissor Lift
    /*
    If  none of the designated buttons are pressed then run without automation.
    ......
    This way you can select how many stages up or down you want the scissor lift to go and then send it that many stone stages.

     */
        public  void LiftControlTest(double y2, boolean left, boolean right, boolean up, boolean down, boolean b2){
            double ScissorLiftPower = Range.clip( (-y2), -1.0, 1.0);
            if(y2 != 0 && !pressed) {
                ScissorLiftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ScissorLiftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ScissorLiftMotorLeft.setPower(ScissorLiftPower);
                ScissorLiftMotorRight.setPower(ScissorLiftPower);
            }
            if(!left && !right && !up && !down){
                pressed = false;
            }
            if(up && !pressed){
                pressed = true;
                pos += 1;
            }
            if(down && !pressed && pos > 0){
                pressed = true;
                pos -= 1;
            }
            if(right && !pressed){
                pressed = true;
                target = ((pos * block) + initial);
                ScissorLiftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ScissorLiftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ScissorLiftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ScissorLiftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ScissorLiftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ScissorLiftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ScissorLiftMotorLeft.setTargetPosition(target);
                ScissorLiftMotorRight.setTargetPosition(target);
                ScissorLiftMotorLeft.setPower(0.6);
                ScissorLiftMotorRight.setPower(0.6);

                while(ScissorLiftMotorLeft.isBusy() || ScissorLiftMotorRight.isBusy()){}
                ScissorLiftMotorLeft.setPower(0);
                ScissorLiftMotorRight.setPower(0);

            }
            if(left && !pressed){
                pressed = true;
                target = ((pos * block) + initial);
                ScissorLiftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ScissorLiftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ScissorLiftMotorLeft.setPower(-0.3);
                ScissorLiftMotorRight.setPower(-0.3);
                while(ScissorLiftMotorLeft.getCurrentPosition() > 0 || ScissorLiftMotorRight.getCurrentPosition() > 0){}
                ScissorLiftMotorLeft.setPower(0);
                ScissorLiftMotorRight.setPower(0);

            }
            if(b2 && !pressed){
                pressed = true;
                target =  initial;
                ScissorLiftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ScissorLiftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ScissorLiftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ScissorLiftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ScissorLiftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ScissorLiftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ScissorLiftMotorLeft.setTargetPosition(initial);
                ScissorLiftMotorRight.setTargetPosition(initial);
                ScissorLiftMotorLeft.setPower(0.6);
                ScissorLiftMotorRight.setPower(0.6);
                while(ScissorLiftMotorLeft.isBusy() || ScissorLiftMotorRight.isBusy()){}
                ScissorLiftMotorLeft.setPower(0);
                ScissorLiftMotorRight.setPower(0);

            }
    }

}
