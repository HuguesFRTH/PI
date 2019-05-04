/**
 *  Ixion
 */
package fr.ixion.pi;

import java.io.IOException;

import fr.ixion.pi.serial.Serial;

/**
 * @author Ixion
 *
 */
public class Command
{

    public double azimut;
    public double hauteur;
    public int astreID;
    
    public Command(int id, double ht, double az) {
        this.astreID = id;
        this.hauteur = ht;
        this.azimut = az;
    }

    
    public void execute() throws IOException {
        Serial.instance.sendPacket(this);
    }


    /**
     * 
     */
    public void talk()
    {
        Utils.log(azimut);

        Utils.log(hauteur);

        Utils.log(astreID);
    }


    /**
     * @return
     */
    public static Command nul()
    {
        // TODO Auto-generated method stub
        return new Command(444,0,0);
    }
}
