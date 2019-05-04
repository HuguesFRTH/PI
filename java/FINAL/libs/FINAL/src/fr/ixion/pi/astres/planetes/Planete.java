/**
 *  Ixion
 */
package fr.ixion.pi.astres.planetes;

import static fr.ixion.pi.Maths.*;

import static fr.ixion.pi.Soleil.*;

import fr.ixion.pi.Location;
import fr.ixion.pi.Soleil;
import fr.ixion.pi.Temps;
import fr.ixion.pi.Utils;
import fr.ixion.pi.astres.Astre;

/**
 * @author Ixion
 */
public abstract class Planete extends Astre
{

    public double const1;
    public double const2;
    public double const3;
    public double const4;
    public double const5;
    public double const6;
    public double const7;
    public double const8;
    public double const9;
    public double const10;
    public double const11;
    public double const12;
    public double const13;
    public double const14;
    public double const15;
    public double const16;
    public double const17;


    public abstract void constantes();

    public Planete()
    {
        constantes();
    }

    public void particularity()
    {
        constantes();
    }
}
