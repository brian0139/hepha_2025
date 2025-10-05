package org.firstinspires.ftc.teamcode.Alvin;

public class intake {
    public int[] slots={0,0,0};
    public void arraySlot() {
        //update array
        int[] tmp = {1,0,1}; //make the array value 0 if there is no ball, 1 if there is a ball
                             //use color sensor to check if there is a ball when intake occurs
        this.slots=tmp;
    }
}
