/**
 *  Ixion
 */
package fr.ixion.pi.astres.planetes;


import static fr.ixion.pi.Maths.*;

import static fr.ixion.pi.Soleil.*;

/**
 * @author Ixion
 */
public class Saturne extends Planete
{

    /*
     * (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#constantes()
     */
    @Override
    public void constantes()
    {
        const1 = 266.564377;
        const2 = 1223.509884;
        const3 = 0.0003245;
        const4 = 9.55491173474;
        const5 = 0.05589232;
        const6 = -0.0003455;
        const7 = -0.000000728;
        const8 = 2.492519;
        const9 = -0.0039189;
        const10 = -0.00001549;
        const11 = 175.46622;
        const12 = 1221.55147;
        const13 = -0.000502;
        const14 = 112.790414;
        const15 = 0.8731951;
        const16 = -0.00015218;
        const17 = 166.194;

    }

    @Override
    public void update()

    {
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

        double granda = (-0.8142 + 0.0181 * u + 0.0167 * u * u) * sin(v);
        granda = granda + (-0.0105 + 0.1609 * u - 0.0041 * u * u) * cos(v);
        granda = granda - 0.1488 * sin(dzeta) - 0.0408 * sin(2 * dzeta) - 0.0152 * sin(3 * dzeta);
        granda = granda + (0.0089 * sin(dzeta) - 0.0165 * sin(2 * dzeta)) * sin(q);
        granda = granda + (0.0813 * cos(dzeta) + 0.015 * cos(2 * dzeta)) * sin(q);
        granda = granda + (0.0856 * sin(dzeta) + 0.0253 * cos(dzeta) + 0.0144 * cos(2 * dzeta)) * cos(q);
        granda = granda + 0.0092 * sin(2 * dzeta) * sin(2 * q);

        // --------perturbations dans l'excentricité

        double grandb = (-793 + 255 * u) * sin(v) + (1338 + 123 * u) * cos(v);
        grandb = grandb + 1241 * sin(q) + (39 - 62 * u) * sin(dzeta) * sin(q);
        grandb = grandb + (2660 * cos(dzeta) - 469 * cos(2 * dzeta) - 187 * cos(3 * dzeta) - 82 * cos(4 * dzeta)) * sin(q);
        grandb = grandb - (1270 * sin(dzeta) + 420 * sin(2 * dzeta) + 150 * sin(3 * dzeta)) * cos(q);
        grandb = grandb - 62 * sin(4 * dzeta) * cos(q);
        grandb = grandb + (221 * sin(dzeta) - 221 * sin(2 * dzeta) - 57 * sin(3 * dzeta)) * sin(2 * q);
        grandb = grandb - (278 * cos(dzeta) - 202 * cos(2 * dzeta)) * sin(2 * q);
        grandb = grandb - (284 * sin(dzeta) + 159 * cos(dzeta)) * cos(2 * q);
        grandb = grandb + (216 * cos(2 * dzeta) + 56 * cos(3 * dzeta)) * cos(2 * q);

        // --------perturbations dans la longitude du périhélie

        double grandc = (0.0771 + 0.0072 * u) * sin(v);
        grandc = grandc + (0.0458 - 0.0148 * u) * cos(v);
        grandc = grandc - (0.0758 * sin(dzeta) + 0.0248 * sin(2 * dzeta) + 0.0086 * sin(3 * dzeta)) * sin(q);
        grandc = grandc - (0.0726 + 0.1504 * cos(dzeta) - 0.0269 * cos(2 * dzeta) - 0.0101 * cos(3 * dzeta)) * cos(q);
        grandc = grandc - (0.0136 * sin(dzeta) - 0.0136 * cos(2 * dzeta)) * sin(2 * q);
        grandc = grandc - (0.0137 * sin(dzeta) - 0.012 * sin(2 * dzeta)) * cos(2 * q);
        grandc = grandc + (0.0149 * cos(dzeta) - 0.0131 * cos(2 * dzeta)) * cos(2 * q);

        // --------perturbations dans le demi-grand axe

        double grandd = 2933 * cos(v) + 33629 * cos(dzeta) - 3081 * cos(2 * dzeta) - 1423 * cos(3 * dzeta) - 671 * cos(4 * dzeta) + (1098 - 2812 * sin(dzeta) + 688 * sin(2 * dzeta)) * sin(q);
        grandd = grandd + (2138 * cos(dzeta) - 999 * cos(2 * dzeta) - 642 * cos(3 * dzeta)) * sin(q) - 890 * cos(q) + (2206 * sin(dzeta) - 1590 * sin(2 * dzeta) - 647 * sin(3 * dzeta)) * cos(q) + (2885 * cos(dzeta) + 2172 * cos(2 * dzeta)) * cos(q) - 778 * cos(dzeta) * sin(2 * q) - 856 * sin(dzeta) * cos(2 * q);

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

        double corr = 0.000747 * cos(dzeta) * sin(q) + 0.001069 * cos(dzeta) * cos(q);
        corr = corr + 0.002108 * sin(2 * dzeta) * sin(2 * q) + 0.001261 * cos(2 * dzeta) * sin(2 * q);
        corr = corr + 0.001236 * sin(2 * dzeta) * cos(2 * q) - 0.002075 * cos(2 * dzeta) * cos(2 * q);
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

        double elongationSat = elongation;
        String elongation_planeteSat = elongation_planete;

        String commentSat;
        String quandSat;
        if(elongationSat < 20)
        {
            commentSat = "Inobservable";;
            quandSat = " ";
        }

        if(elongationSat > 20 && elongationSat < 45)
        {
            commentSat = "Observable";
            if(elongation_planeteSat == "Ouest")
            {
                quandSat = "en toute fin de nuit";
            }
            if(elongation_planeteSat == "Est")
            {
                quandSat = "en tout début de soirée";
            }
        }

        if(elongationSat > 45 && elongationSat < 120)
        {
            commentSat = "Observable";
            if(elongation_planeteSat == "Ouest")
            {
                quandSat = "en seconde partie de nuit";
            }
            if(elongation_planeteSat == "Est")
            {
                quandSat = "en première partie de nuit";
            }
        }

        if(elongationSat > 120 && elongationSat < 140)
        {
            commentSat = "Observable";
            quandSat = "pratiquement toute la nuit";
        }
        if(elongationSat > 140 && elongationSat < 180)
        {
            commentSat = "Observable";
            quandSat = "toute la nuit";
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

        double ADSat = asc * 15 / r2d;

        d = abs(declin);
        d1 = ((d - floor(d)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        double DecSat;
        if(declin < 0)
        {
            signe = "-";
            DecSat = -d / r2d;
        }
        else
        {
            signe = "+";
            DecSat = +d / r2d;
        }

       double ascention = asc;
       double déclinaison = declin;

        // Magnitude de la planète

        double dist = R;               // rayon vecteur Soleil-Terre
        double ray = r;                   // rayon vecteur Soleil-planète
        double delta = distance_terre;   // distance Terre-planète

        double FV = acos((ray * ray + delta * delta - dist * dist) / (2 * ray * delta));
        FV = (FV) * r2d;

        double phase = (1 + cos(FV / r2d)) * 50;

        double magnitude = -9.0 + 5 * (log(ray * delta)) / log(10) + 0.044 * FV;
        magnitude = floor(magnitude * 100) / 100;

        double los = l1;  // geocentric ecliptic longitude
        double las = b1;   // geocentric ecliptic latitude

        double ir = 28.06 / r2d;
        double Nr = (169.51 / r2d) + (3.82E-5 / r2d) * T;
        double B = asin(sin(las) * cos(ir) - cos(las) * sin(ir) * sin(los - Nr));

        double ring_magn = -2.6 * sin(abs(B)) + 1.2 * pow(sin(B), 2);
        ring_magn = floor(ring_magn * 100) / 100;

    }
    @Override
    public int getID()
    {
        // TODO Auto-generated method stub
        return 5;
    }

}

