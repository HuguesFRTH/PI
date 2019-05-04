/**
 *  Ixion
 */
package fr.ixion.pi.astres.planetes;

import static fr.ixion.pi.Maths.*;

import static fr.ixion.pi.Soleil.*;

/**
 * @author Ixion
 */
public class Uranus extends Planete
{

    /*
     * (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#constantes()
     */
    @Override
    public void constantes()
    {
        const1 = 244.19747;
        const2 = 429.863546;
        const3 = 0.000316;
        const4 = 19.21844609894;
        const5 = 0.0463444;
        const6 = -0.00002658;
        const7 = 0.000000077;
        const8 = 0.772464;
        const9 = 0.0006253;
        const10 = 0.0000395;
        const11 = 72.648778;
        const12 = 428.3791132;
        const13 = 0.0000788;
        const14 = 73.477111;
        const15 = 0.4986678;
        const16 = 0.0013117;
        const17 = 70.481;

    }

    @Override
    public int getID()
    {
        // TODO Auto-generated method stub
        return 6;
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
        double g = (83.76922 + 218.4901 * T) / r2d;
        double h = (284.02 + 8.51 * T) / r2d;
        double s = (243.52 + 428.47 * T) / r2d;
        double w = 2 * p - 6 * q + 3 * s;
        double dzeta = s - p;
        double eta = s - q;
        double theta = (200.25 - 209.98 * T) / r2d;

        // --------perturbations

        double granda = (0.864319 - 0.001583 * u) * sin(h) + 0.036017 * sin(2 * h) + (0.082222 - 0.006833 * u) * cos(h) - 0.003019 * cos(2 * h) + 0.008122 * sin(w);
        double grandb = 2098 * cos(h) - 335 * sin(h) + 131 * cos(2 * h);
        double grandc = 0.120303 * sin(h) + (0.019472 - 0.000947 * u) * cos(h) + 0.006197 * sin(2 * h);
        double grandd = -3825 * cos(h);

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

        r = r - 0.025948 + 0.004985 * cos(dzeta) - 0.00123 * cos(s) + 0.003354 * cos(eta);
        r = r + (0.005795 * cos(s) - 0.001165 * sin(s) + 0.001388 * cos(2 * s)) * sin(eta);
        r = r + (0.001351 * cos(s) + 0.005702 * sin(s) + 0.00138 * sin(2 * s)) * cos(eta);
        r = r + 0.000904 * cos(2 * theta) + 0.000894 * (cos(theta) - cos(3 * theta));

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

        l = l + ((0.010122 - 0.000988 * u) * sin(s + eta) + (-0.038581 + 0.002031 * u - 0.00191 * u * u) * cos(s + eta) + (0.034964 - 0.001038 * u + 0.000868 * u * u) * cos(2 * s + eta) - 0.014808 * sin(dzeta) - 0.005794 * sin(eta) + 0.002347 * cos(eta) + 0.009872 * sin(theta) + 0.008803 * sin(2 * theta) - 0.004308 * sin(3 * theta)) / r2d;

        if(l > 2 * PI)
        {
            l = l - 2 * PI;
        }

        double b = asin(sin(u) * sin(i));

        double corr = (0.000458 * sin(eta) - 0.000642 * cos(eta) - 0.000517 * cos(4 * theta)) * sin(s);
        corr = corr - (0.000347 * sin(eta) + 0.000853 * cos(eta) + 0.000517 * sin(4 * eta)) * cos(s);
        corr = corr + 0.000403 * (cos(2 * theta) * sin(2 * s) + sin(2 * theta) * cos(2 * s));
        b = b + corr / r2d;

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

        double elongationUran = elongation;
        String elongation_planeteUran = elongation_planete;

        String quandUran;
        String commentUran;
        if(elongationUran < 20)
        {
            commentUran = "Inobservable";
            quandUran = " ";
        }

        if(elongationUran > 20 && elongationUran < 45)
        {
            commentUran = "Observable";
            if(elongation_planeteUran == "Ouest")
            {
                quandUran = "en toute fin de nuit";
            }
            if(elongation_planeteUran == "Est")
            {
                quandUran = "en tout début de soirée";
            }
        }

        if(elongationUran > 45 && elongationUran < 120)
        {
            commentUran = "Observable";
            if(elongation_planeteUran == "Ouest")
            {
                quandUran = "en seconde partie de nuit";
            }
            if(elongation_planeteUran == "Est")
            {
                quandUran = "en première partie de nuit";
            }
        }

        if(elongationUran > 120 && elongationUran < 140)
        {
            commentUran = "Observable";
            quandUran = "pratiquement toute la nuit";
        }
        if(elongationUran > 140 && elongationUran < 180)
        {
            commentUran = "Observable";
            quandUran = "toute la nuit";
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

        double ADUran = asc * 15 / r2d;

        d = abs(declin);
        d1 = ((d - floor(d)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        double DecUran;
        if(declin < 0)
        {
            signe = "-";
            DecUran = -d / r2d;
        }
        else
        {
            signe = "+";
            DecUran = +d / r2d;
        }

        double ascUran = asc;
        double declinUran = declin;

        // Magnitude de la planète

        double dist = R;               // rayon vecteur Soleil-Terre
        double ray = r;                     // rayon vecteur Soleil-planète
        double delta = distance_terre;   // distance Terre-planète

        double FV = acos((ray * ray + delta * delta - dist * dist) / (2 * ray * delta));
        FV = (FV) * r2d;
        double phase = (1 + cos(FV / r2d)) * 50;

        double magnitude = -7.15 + 5 * (log(ray * delta)) / log(10) + 0.001 * FV;
        magnitude = ceil(magnitude * 100) / 100;
    }

}
