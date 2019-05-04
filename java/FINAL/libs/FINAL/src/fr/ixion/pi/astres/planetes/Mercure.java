/**
 *  Ixion
 */
package fr.ixion.pi.astres.planetes;

import static fr.ixion.pi.Maths.*;
import static fr.ixion.pi.Soleil.*;

/**
 * @author Ixion
 *
 */
public class Mercure extends Planete
{

    /* (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#constantes()
     */
    @Override
    public void constantes()
    {
         const1 = 178.179078;
         const2 = 149474.07078;
         const3 = 0.0003011;
         const4 = 0.38709830982;
         const5 = 0.20561421;
         const6 = 0.00002046;
         const7 = -0.00000003;
         const8 = 7.002881;
         const9 = 0.0018608;
         const10 = -0.0000183;
         const11 = 102.27938;
         const12 = 149472.51529;
         const13 = 0.000007;
         const14 = 47.145944;
         const15 = 1.1852083;
         const16 = 0.0001739;
         const17 = 6.728;
    }
    @Override
    public int getID()
    {
        // TODO Auto-generated method stub
        return 2;
    }
    /* (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#update()
     */
    @Override
    public void update()
    {
        // --------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i),
        // --------longitude noeud ascendant (m)

        double l = (const1 + const2 * T + const3 * T2) / r2d;
        l = l - y * floor(l / y);

        double a = const4;

        e = const5 + const6 * T + const7 * T2;
        double i = (const8 + const9 * T + const10 * T2) / r2d;
        double m = (const11 + const12 * T + const13 * T2) / r2d;
        m = m - y * floor(m / y);
        double longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;

        // --------------- Anomalie moyenne

        double m1 = (102.27938 + 149472.51529 * T + 0.000007 * T2) / r2d;
        double m2 = (212.60322 + 58517.80387 * T + 0.001286 * T2) / r2d;
        double m4 = (319.51913 + 19139.85475 * T + 0.000181 * T2) / r2d;
        double m5 = (225.32833 + 3034.69202 * T - 0.000722 * T2) / r2d;
        double m6 = (175.46622 + 1221.55147 * T - 0.000502 * T2) / r2d;

        m1 = m1 - y * floor(m1 / y);
        m2 = m2 - y * floor(m2 / y);
        m4 = m4 - y * floor(m4 / y);
        m5 = m5 - y * floor(m5 / y);
        m6 = m6 - y * floor(m6 / y);

        // --------équation de Kepler

        double grand_e = m;
        grand_e = m + e * sin(grand_e);
        grand_e = m + e * sin(grand_e);
        grand_e = m + e * sin(grand_e);
        grand_e = m + e * sin(grand_e);
        grand_e = m + e * sin(grand_e);
        grand_e = m + e * sin(grand_e);
        grand_e = m + e * sin(grand_e);
        grand_e = m + e * sin(grand_e);
        grand_e = m + e * sin(grand_e);
        grand_e = m + e * sin(grand_e);

        // --------anomalie vraie

        v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(grand_e / 2));
        if(v < 0)
        {
            v = v + 2 * PI;
        }

        // --------rayon vecteur

        double r = a * (1 - e * cos(grand_e));

        r = r + 0.000007525 * cos(2 * m5 - m1 + 53.013 / r2d);
        r = r + 0.000006802 * cos(5 * m2 - 3 * m1 - 259.918 / r2d);
        r = r + 0.000005457 * cos(2 * m2 - 2 * m1 - 71.188 / r2d);
        r = r + 0.000003569 * cos(5 * m2 - m1 - 77.75 / r2d);

        // --------argument de latitude

        double u = l + v - m - longitude_noeud;
        u = u - y * floor(u / y);

        if(cos(u) != 0)
        {
            d = atan(cos(i) * tan(u));
            if(cos(u) < 0)
            {
                d = d + PI;
            }
        }
        else
        {
            d = u;
        }

        // --------longitude écliptique

        l = d + longitude_noeud;

        l = l + 0.00204 / r2d * cos(5 * m2 - 2 * m1 + 12.22 / r2d);
        l = l + 0.00103 / r2d * cos(2 * m2 - m1 - 160.692 / r2d);
        l = l + 0.00091 / r2d * cos(2 * m5 - m1 - 37.003 / r2d);
        l = l + 0.00078 / r2d * cos(5 * m2 - 3 * m1 + 10.137 / r2d);

        if(l > 2 * PI)
        {
            l = l - 2 * PI;
        }

        double b = asin(sin(u) * sin(i));

        double numerateur = r * cos(b) * sin(l - longitude_terre + PI);
        double denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;

        l = atan(numerateur / denominateur) + longitude_terre + PI;

        if(l > 2 * PI)
        {
            l = l - 2 * PI;
        }

        if(denominateur < 0)
        {
            l = l + PI;
        }

        // --------conversion rectangulaire/polaire

        cos(b);
        cos(l);
        cos(latitude_terre);
        cos(longitude_terre);
        cos(b);
        sin(l);
        cos(latitude_terre);
        sin(longitude_terre);
        sin(b);
        sin(latitude_terre);

        double distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));

        // --------élongation

        omega = 259.18 / r2d - 1934.142 / r2d * T;
        l = l - 0.00479 / r2d * sin(omega);

        double elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
        String elongation_planete;
        if((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI))
        {
            elongation_planete = "Ouest";
        }
        else
        {
            elongation_planete = "Est";
        }

        double elongationMerc = elongation;
        String elongation_planeteMerc = elongation_planete;
        if(elongationMerc < 10)
        {}

        if(elongationMerc > 10 && elongationMerc < 20)
        {
            if(elongation_planeteMerc == "Ouest")
            {}
            if(elongation_planeteMerc == "Est")
            {}
        }

        if(elongationMerc > 20 && elongationMerc < 50)
        {
            if(elongation_planeteMerc == "Ouest")
            {}
            if(elongation_planeteMerc == "Est")
            {}
        }

        // --------convertion longitude et latitude en ascension droite et déclinaison

        l = l - y * floor(l / y);
        double l1 = l * r2d;

        d1 = ((l1 - floor(l1)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        double beta = asin(r * sin(b) / distance_terre);

        double b1 = abs(beta * r2d);
        d1 = ((b1 - floor(b1)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        String signe;
        if(beta < 0)
        {
            signe = "-";
        }
        else
        {
            signe = "+";
        }

        asc = atan((cos(obliquite) * sin(l) - tan(beta) * sin(obliquite)) / cos(l));
        if(asc < 0)
        {
            asc = asc + 2 * PI;
        }
        if(cos(l) < 0)
        {
            asc = asc + PI;
        }
        if(asc > 2 * PI)
        {
            asc = asc - 2 * PI;
        }
        declin = r2d * asin(sin(beta) * cos(obliquite) + cos(beta) * sin(obliquite) * sin(l));

        asc = asc * r2d / 15;

        d1 = ((asc - floor(asc)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        d = abs(declin);
        d1 = ((d - floor(d)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        if(declin < 0)
        {
            signe = "-";
        }
        else
        {
            signe = "+";
        }

        ascention = asc;
        déclinaison = declin;

        // Magnitude de la planète

        double dist = R;           // rayon vecteur Soleil-Terre
        double ray = r;                  // rayon vecteur Soleil-planète
        double delta = distance_terre;// distance Terre-planète

        double FV = acos((ray * ray + delta * delta - dist * dist) / (2 * ray * delta));
        FV = (FV) * r2d;

        cos(FV / r2d);

        double magnitude = -0.36 + 5 * (log(ray * delta)) / log(10) + 0.027 * FV + 2.2E-13 * pow(FV, 6);
        magnitude = ceil(magnitude * 10) / 10; 
    }

    
}
