/**
 *  Ixion
 */
package fr.ixion.pi.astres;

import java.util.ArrayList;
import java.util.List;

import fr.ixion.pi.Command;
import fr.ixion.pi.astres.planetes.Jupiter;
import fr.ixion.pi.astres.planetes.Mars;
import fr.ixion.pi.astres.planetes.Mercure;
import fr.ixion.pi.astres.planetes.Pluton;

/**
 * @author Ixion
 */
public class AstreManager
{
    public static AstreManager instance;
    public List<Astre> astreList = new ArrayList<>();

    public AstreManager()
    {
        instance = this;

        addAstre(new Jupiter());
        addAstre(new Mars());
        addAstre(new Mercure());
        
        
        addAstre(new Pluton());
        addAstre(new Mercure());
        addAstre(new Mercure());
        addAstre(new Mercure());
        addAstre(new Mercure());
        


    }

    
    public void addAstre(Astre astre) {
        astreList.add(astre);
    }
    public Command returnCMDForID(int id)
    {
        Command cmd = Command.nul();
        for(Astre astre : astreList)
        {
            if(astre.getID() == id)
            {
                cmd = new Command(astre.getID(), astre.hauteur, astre.azimut);
            }
        }
        return cmd;
    }
}
