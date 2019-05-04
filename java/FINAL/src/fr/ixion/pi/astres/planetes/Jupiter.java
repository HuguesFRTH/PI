/**
 *  Ixion
 */
package fr.ixion.pi.astres.planetes;

import static fr.ixion.pi.Maths.*;
import static fr.ixion.pi.Soleil.*;

/**
 * @author Ixion
 */
public class Jupiter extends Planete
{

    /*
     * (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#constantes()
     */
    @Override
    public void constantes()
    {
        const1 = 238.049257;
        const2 = 3036.301986;
        const3 = 0.0003347;
        const4 = 5.20260300002;
        const5 = 0.04833475;
        const6 = 0.00016418;
        const7 = -0.0000004676;
        const8 = 1.308736;
        const9 = -0.0056961;
        const10 = 0.0000039;
        const11 = 225.32833;
        const12 = 3034.69202;
        const13 = -0.000722;
        const14 = 99.443414;
        const15 = 1.01053;
        const16 = 0.00035222;
        const17 = 197.146;
    }

    /*
     * (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#setID()
     */
    @Override
    public int getID()
    {
        // TODO Auto-generated method stub
        return 4;
    }

    /*
     * (non-Javadoc)
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

        double e = const5 + const6 * T + const7 * T2;
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

        // --------Termes périodiques

        double u = T / 5 + 0.1;
        double p = (237.475 + 3034.9061 * T) / r2d;
        double q = (265.916 + 1222.1139 * T) / r2d;
        v = 5 * q - 2 * p;
        double dzeta = q - p;

        // --------perturbations dans la longitude moyenne

        double granda = (0.3314 - 0.0103 * u - 0.0047 * u * u) * sin(v);
        granda = granda + (0.0032 - 0.0644 * u + 0.0021 * u * u) * cos(v);
        granda = granda + 0.0136 * sin(dzeta) + 0.0185 * sin(2 * dzeta) + 0.0067 * sin(3 * dzeta);
        granda = granda + (0.0073 * sin(dzeta) + 0.0064 * sin(2 * dzeta) - 0.0338 * cos(dzeta)) * sin(q);
        granda = granda - (0.0357 * sin(dzeta) + 0.0063 * cos(dzeta) + 0.0067 * cos(2 * dzeta)) * cos(q);

        // --------perturbations dans l'excentricité

        double grandb = (361 + 13 * u) * sin(v) + (129 - 58 * u) * cos(v);
        grandb = grandb + (128 * cos(dzeta) - 676 * sin(dzeta) - 111 * sin(2 * dzeta)) * sin(q);
        grandb = grandb + (146 * sin(dzeta) - 82 + 607 * cos(dzeta) + 99 * cos(2 * dzeta) + 51 * cos(3 * dzeta)) * cos(q);
        grandb = grandb - (96 * sin(dzeta) + 100 * cos(dzeta)) * sin(2 * q) - (96 * sin(dzeta) - 102 * cos(dzeta)) * cos(2 * q);

        // --------perturbations dans la longitude du périhélie

        double grandc = (0.0072 - 0.0031 * u) * sin(v) - 0.0204 * cos(v);
        grandc = grandc + (0.0073 * sin(dzeta) + 0.034 * cos(dzeta) + 0.0056 * cos(2 * dzeta)) * sin(q);
        grandc = grandc + (0.0378 * sin(dzeta) + 0.0062 * sin(2 * dzeta) - 0.0066 * cos(dzeta)) * cos(q);
        grandc = grandc - 0.0054 * sin(dzeta) * sin(2 * q) + 0.0055 * cos(dzeta) * cos(2 * q);

        // --------perturbations dans le demi-grand axe

        double grandd = -263 * cos(v) + 205 * cos(dzeta) + 693 * cos(2 * dzeta) + 312 * cos(3 * dzeta) + 299 * sin(dzeta) * sin(q) + (204 * sin(2 * dzeta) - 337 * cos(dzeta)) * cos(q);

        // --------corrections

        l = l + granda / r2d;
        m = m + granda / r2d - grandc / r2d / e;
        e = e + grandb / 1000000;
        a = a + grandd / 1000000;

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

        // --------argument de latitude

        u = l + v - m - longitude_noeud;
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

        double diametre = const17;

        // --------conversion rectangulaire/polaire

        double xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
        double yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
        double zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);

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

        double elongationJup = elongation;
        String elongation_planeteJup = elongation_planete;

        String commentJup;
        String quandJup;
        if(elongationJup < 20)
        {
            commentJup = "Inobservable";
            quandJup = " ";
        }

        if(elongationJup > 20 && elongationJup < 45)
        {
            commentJup = "Observable";
            if(elongation_planeteJup == "Ouest")
            {
                quandJup = "en toute fin de nuit";
            }
            if(elongation_planeteJup == "Est")
            {
                quandJup = "en tout début de soirée";
            }
        }

        if(elongationJup > 45 && elongationJup < 120)
        {
            commentJup = "Observable";
            if(elongation_planeteJup == "Ouest")
            {
                quandJup = "en seconde partie de nuit";
            }
            if(elongation_planeteJup == "Est")
            {
                quandJup = "en première partie de nuit";
            }
        }

        if(elongationJup > 120 && elongationJup < 140)
        {
            commentJup = "Observable";
            quandJup = "pratiquement toute la nuit";
        }
        if(elongationJup > 140 && elongationJup < 180)
        {
            commentJup = "Observable";
            quandJup = "toute la nuit";
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

        double ADJup = asc * 15 / r2d;

        d = abs(declin);
        d1 = ((d - floor(d)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        double DecJup;
        if(declin < 0)
        {
            signe = "-";
            DecJup = -d / r2d;
        }
        else
        {
            signe = "+";
            DecJup = +d / r2d;
        }

        double ascJup = asc;
        double declinJup = declin;

        // Magnitude de la planète

        double dist = R;              // rayon vecteur Soleil-Terre
        double ray = r;             // rayon vecteur Soleil-planète
        double delta = distance_terre;      // distance Terre-planète

        double FV = acos((ray * ray + delta * delta - dist * dist) / (2 * ray * delta));
        FV = (FV) * r2d;

        double phase = (1 + cos(FV / r2d)) * 50;

        double magnitude = -9.25 + 5 * (log(ray * delta)) / log(10) + 0.014 * FV;
        magnitude = ceil(magnitude * 10) / 10;
    }

}
