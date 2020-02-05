package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;


public class BotServos {
    //Set Up Variables and Servo Devices
    Servo left_hook, right_hook;

    double lStored = 0; // Position for Servo to be out of the way
    double rStored = 1; // Position for Servo to be out of the way
    double lActive = 0.6; //Position for Servo in order to grab and move the foundation
    double rActive = 0.4; //Position for Servo in order to grab and move the foundation

    //Constructor for Bot Servos / Foundation Servos
    public BotServos(Servo lHook, Servo rHook){
        left_hook = lHook;
        right_hook = rHook;
    }

    //Method for retracting the Foundation Servos so they are stored  with either button entered
        public void retract(boolean a1, boolean b1) {
            if (a1 || b1) {
                left_hook.setPosition(lStored);
                right_hook.setPosition(rStored);

            }
        }
        //Method for retracting the Foundation Servos so they are stored with the singular button pressed
            public void retract ( boolean a1){
                if (a1 ) {
                    left_hook.setPosition(lStored);
                    right_hook.setPosition(rStored);

                }
            }

            // Method for using the Foundation Servos in autonomous.
    // If true, set the servos to be able to grab the foundation
    // If false, set the servos to their stored position.
            public void auto(boolean a2){
                if(a2){
                    left_hook.setPosition(lActive);
                    right_hook.setPosition(rActive);
                }
                else{
                    left_hook.setPosition(lStored);
                    right_hook.setPosition(rStored);
                }
            }

            //Method for setting the Foundation Servos to their active position where they can
            // grab the foundation with the use of either button entered.
            public void activate ( boolean y1, boolean y2){
                if (y1 || y2) {
                    left_hook.setPosition(lActive);
                    right_hook.setPosition(rActive);

                }

            }
            //Method for setting the Foundation Servos to their active, foundation-grabbing
            // setting with the singular button entered.
            public void activate ( boolean y1){
                if (y1 ) {
                    left_hook.setPosition(lActive);
                    right_hook.setPosition(rActive);

                }
            }



}
