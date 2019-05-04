/**
 *  Ixion
 */
package fr.ixion.pi.astres;

import static fr.ixion.pi.Maths.asin;
import static fr.ixion.pi.Maths.atan;
import static fr.ixion.pi.Maths.cos;
import static fr.ixion.pi.Maths.d2r;
import static fr.ixion.pi.Maths.floor;
import static fr.ixion.pi.Maths.r2d;
import static fr.ixion.pi.Maths.sin;
import static fr.ixion.pi.Maths.tan;

import fr.ixion.pi.Location;
import fr.ixion.pi.Soleil;
import fr.ixion.pi.Temps;
import fr.ixion.pi.Utils;

/**
 * @author Ixion
 */
public abstract class Astre
{
    

    public double azimut;
    public double hauteur;
    public double asc;
    public double declin;
    
    public Astre()
    {
        particularity();
        updateAll();
        convertion();
        talk();
    }

    public void particularity() {
        
    }
    public abstract int getID();

    public void convertion()
    {
        // CONVERSION

        double latitude = Location.instance.latitude;

        double Ho = 15 * (Temps.instance.TS + Temps.instance.date.heures + (double)((double)Temps.instance.date.minutes / (double)60) - asc) + (-Location.instance.longitude * d2r);

        double num = sin(-Ho * d2r);
        double denom = (cos(-Ho * d2r) * sin(-latitude * d2r) - tan(-declin * d2r) * cos(-latitude * d2r));
        azimut = (double)num / (double)denom;
        azimut = 180 - atan(-azimut) * r2d;
        hauteur = r2d * asin(sin(declin * d2r) * sin(latitude * d2r) + cos(declin * d2r) * cos(Ho * d2r) * cos(latitude * d2r));

    }

    /**
    * 
    */
    public void talk()
    {
        // Utils.log(getClass().getSimpleName() + " -> " + "asc : " + ascention + " declin : " + déclinaison);
        Utils.log(getClass().getSimpleName() + " -> " + " ht : " + ((int)floor(hauteur)) + "°" + (int)(((hauteur - (floor(hauteur))) * 60)) + "'" + " az : " + ((int)floor(azimut)) + "°" + (int)(((azimut - (floor(azimut))) * 60)) + "'");
    }

    public void updateAll()
    {
        Soleil.update();
        update();
        convertion();
    }

    public abstract void update();
}
