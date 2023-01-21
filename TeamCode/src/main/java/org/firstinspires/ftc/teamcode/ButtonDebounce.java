package org.firstinspires.ftc.teamcode;


public class ButtonDebounce {
    long lastChange = 0;
    boolean previousState = false;
    boolean sentButton = false;
    
    public boolean getButton(boolean button) {
        if (lastChange == 0) {
            lastChange = System.nanoTime();
        }
        
        if (button == previousState && (System.nanoTime() - lastChange >= 10000) && !sentButton && button == true) {
            sentButton = true;
            return true;
        
         } else if (button != previousState) {
            lastChange = System.nanoTime();
            previousState = button;
            sentButton = false;
        }
        
        
        return false;
        
        
    }
        
    
    // todo: write your code here
}