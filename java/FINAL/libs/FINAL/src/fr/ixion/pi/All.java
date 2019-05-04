/**
 *  Ixion
 */
package fr.ixion.pi;

import static fr.ixion.pi.Maths.*;

import javax.rmi.CORBA.Util;

/**
 * @author Ixion
 */
public class All
{

    double PI = Math.PI;
    double DR = PI / 180;
    double K1 = 15 * DR * 1.0027379;
    boolean Moonrise = false;
    boolean Moonset = false;
    double[] Rise_time = {0, 0};
    double[] Set_time = {0, 0};
    double Rise_az = 0.0;
    double Set_az = 0.0;
    double[] Sky = {0.0, 0.0, 0.0};
    double[] RAn = {0.0, 0.0, 0.0};
    double[] Dec = {0.0, 0.0, 0.0};
    double[] VHz = {0.0, 0.0, 0.0};

    double nst1 = 1;
    double nst2 = 1;

    double time = 2000;
    private String signe;

    public All()
    {
        date();
        calculer();
    }

    public void date()
    {

        double jour = (int)Temps.instance.date.d;
        double mois = Temps.instance.date.m;
        double annee = Temps.instance.date.y;
        double heure = Temps.instance.date.heures;
        double minute = Temps.instance.date.minutes;
        double seconde = Temps.instance.date.secondes;
    }

    @SuppressWarnings("unused")
    public void calculer()
    {

        // --------------- Données pour circonstances locales

        double latdeg = Location.instance.latitude;
        double latmin = 0;
        double londeg = Location.instance.longitude;
        double lonmin = 0;

        double T = Temps.instance.T;

        double T2 = Temps.instance.T2;

        double T3 = Temps.instance.T3;

        // --------------- Longitude moyenne du Soleil, rapportée à l'équinoxe moyen de la date considérée

        double Lo = 279.69668 + 36000.76892 * T + 0.0003025 * T2;
        Lo = Lo / r2d;

        // --------------- Anomalie moyenne du Soleil

        double M = (358.47583 + 35999.04975 * T) - 0.00015 * T2 - 0.0000033 * T3;
        M = M / r2d;
        double anomalie_soleil = M;

        // --------------- Excentricité de l'orbite terrestre

        double e = 0.01675104 - 0.0000418 * T - 0.000000126 * T2;

        // --------------- Equation de centre C du Soleil

        double c = (1.919460 - 0.004789 * T - 0.000014 * T2) * sin(M) + (0.020094 - 0.000100 * T) * sin(2 * M) + 0.000293 * sin(3 * M);
        c = c / r2d;

        // --------------- Longitude vraie du Soleil rapportée à l'équinoxe moyen de la date considéré, Anomalie vraie

        double Lov = Lo + c;
        double v = M + c;

        // --------------- Rayon vecteur

        double R = 1.0000002 * (1 - e * e) / (1 + e * cos(v));

        // --------------- Longitude apparente du Soleil rapportée à l'équinoxe vrai de l'époque

        double omega = 259.18 / r2d - 1934.142 / r2d * T;
        double longapp = Lov - 0.00569 / r2d - 0.00479 / r2d * sin(omega);

        double longapp0 = longapp - y * floor(longapp / y);
        double longapp1 = longapp0 * r2d;

        double d1 = ((longapp1 - floor(longapp1)) * 60);
        double d2 = ((d1 - floor(d1)) * 60);

        // ---------------Corrections à appliquer à la longitude solaire et au rayon vecteur pour une meilleur précision :
        // ------- A et B sont les corrections dues à l'action de Vénus, C à celle de Jupiter,
        // ------- D à celle de la Lune, alors que E est une inégalité de longue période

        double A = 153.23 / r2d + 22518.7541 / r2d * T;
        double B = 216.57 / r2d + 45037.5082 / r2d * T;
        double C = 312.69 / r2d + 32964.3577 / r2d * T;
        double D = 350.74 / r2d + 445267.1142 / r2d * T - 0.00144 / r2d * T2;
        double E = 231.19 / r2d + 20.20 / r2d * T;
        double H = 353.40 / r2d + 65928.7155 / r2d * T;

        double longitude = longapp + 0.00134 / r2d * cos(A) + 0.00154 / r2d * cos(B) + 0.002 / r2d * cos(C) + 0.00179 / r2d * sin(D) + 0.00178 / r2d * sin(E);
        double longitude1 = longapp - y * floor(longapp / y);
        double long1 = longitude1 * r2d;

        d1 = ((long1 - floor(long1)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        R = R + 0.00000543 * sin(A) + 0.00001575 * sin(B) + 0.00001627 * sin(C) + 0.00003076 * cos(D) + 0.00000927 * sin(H);

        // --------------- Obliquité de l'écliptique

        double obliquite = (23.452294 - 0.0130125 * T - 0.00000164 * T2 + 0.000000503 * T3 + 0.00256 * cos(omega)) / r2d;
        double obliquite1 = (23.452294 - 0.0130125 * T - 0.00000164 * T2 + 0.000000503 * T3) / r2d;

        // --------------- Equation du Temps (formule de W.M. SMART - "Text-Book on Spherical Astronomy" page 19 - édition de 1956)

        double eps = obliquite;

        double yT = tan(eps / 2) * tan(eps / 2);

        double eqT = -yT * sin(2 * Lo) + 2 * e * sin(M) - 4 * e * yT * sin(M) * cos(2 * Lo) + 0.5 * yT * yT * sin(4 * Lo) + 5 / 4 * e * e * sin(2 * M);
        eqT = eqT * r2d;
        // --------------- Coordonnées du Soleil : Ascension droite

        double asc = r2d / 15 * atan(cos(obliquite) * sin(longitude) / cos(longitude));

        if(asc < 0)
        {
            asc = asc + 24;
        }
        if(cos(longitude) < 0)
        {
            asc = asc + 12;
        }
        if(asc > 24)
        {
            asc = asc - 24;
        }

        d1 = ((asc - floor(asc)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        double ADSoleil = asc * 15 / r2d;

        // --------------- Coordonnées du Soleil : Déclinaison

        double declin = r2d * asin(sin(obliquite) * sin(longitude));

        double d = abs(declin);
        d1 = ((d - floor(d)) * 60);
        d2 = ((d1 - floor(d1)) * 60);
        double DecSoleil = 0;
        if(declin < 0)
        {
            DecSoleil = -d / r2d;
        }
        else
        {
            DecSoleil = +d / r2d;
        }

        double ascSoleil = asc;
        double declinSoleil = declin;

        // --------------------------------------------- Calcul de la position des planètes

        // longitude_soleil = Lov - y * floor(Lov / y);
        double longitude_soleil = longitude - y * floor(longitude / y);
        double longitude_terre = longitude_soleil + PI;

        double latitude_terre = 0;

        double distance_terre_soleil = R;

        double longitude_vraie = longitude_soleil;

        // --------------- Coordonnées rectangulaires équatoriales du Soleil

        double xs = R * cos(longitude_vraie);
        double ys = R * sin(longitude_vraie) * cos(obliquite);
        double zs = R * sin(longitude_vraie) * sin(obliquite);

        
      //TODO  MERCURE
        
        // --------------------------------------------- Mercure

        double const1 = 178.179078;
        double const2 = 149474.07078;
        double const3 = 0.0003011;
        double const4 = 0.38709830982;
        double const5 = 0.20561421;
        double const6 = 0.00002046;
        double const7 = -0.00000003;
        double const8 = 7.002881;
        double const9 = 0.0018608;
        double const10 = -0.0000183;
        double const11 = 102.27938;
        double const12 = 149472.51529;
        double const13 = 0.000007;
        double const14 = 47.145944;
        double const15 = 1.1852083;
        double const16 = 0.0001739;
        double const17 = 6.728;

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

        double elongationMerc = elongation;
        String elongation_planeteMerc = elongation_planete;
        String commentMerc = "";
        String quandMerc = "";
        if(elongationMerc < 10)
        {
            commentMerc = "Inobservable";
            quandMerc = " ";
        }

        if(elongationMerc > 10 && elongationMerc < 20)
        {
            commentMerc = "Difficilement visible";
            if(elongation_planeteMerc == "Ouest")
            {
                quandMerc = "peu avant le lever du Soleil";
            }
            if(elongation_planeteMerc == "Est")
            {
                quandMerc = "au coucher du Soleil";
            }
        }

        if(elongationMerc > 20 && elongationMerc < 50)
        {
            commentMerc = "Visible";
            if(elongation_planeteMerc == "Ouest")
            {
                quandMerc = "dans les lueurs de l'aube";
            }
            if(elongation_planeteMerc == "Est")
            {
                quandMerc = "au crépuscule";
            }
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

        double ADMerc = asc * 15 / r2d;

        d = abs(declin);
        d1 = ((d - floor(d)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        double DecMerc;
        if(declin < 0)
        {
            signe = "-";
            DecMerc = -d / r2d;
        }
        else
        {
            signe = "+";
            DecMerc = +d / r2d;
        }

        double ascMerc = asc;
        double declinMerc = declin;
        Utils.log(ascMerc);
        Utils.log(declinMerc);

        // Magnitude de la planète

        double dist = R;           // rayon vecteur Soleil-Terre
        double ray = r;                  // rayon vecteur Soleil-planète
        double delta = distance_terre;// distance Terre-planète

        double FV = acos((ray * ray + delta * delta - dist * dist) / (2 * ray * delta));
        FV = (FV) * r2d;

        double phase = (1 + cos(FV / r2d)) * 50;

        double magnitude = -0.36 + 5 * (log(ray * delta)) / log(10) + 0.027 * FV + 2.2E-13 * pow(FV, 6);
        magnitude = ceil(magnitude * 10) / 10;
        
      //TODO  VENUS

        // --------------------------------------------- Vénus

        const1 = 342.767053;
        const2 = 58519.21191;
        const3 = 0.0003097;
        const4 = 0.72332981996;
        const5 = 0.00682069;
        const6 = -0.00004774;
        const7 = 0.000000091;
        const8 = 3.393631;
        const9 = 0.0010058;
        const10 = -0.000001;
        const11 = 212.60322;
        const12 = 58517.80387;
        const13 = 0.001286;
        const14 = 75.779647;
        const15 = 0.89985;
        const16 = 0.00041;
        const17 = 16.688;

        // --------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i),
        // --------longitude noeud ascendant (m)

        l = (const1 + const2 * T + const3 * T2) / r2d;
        l = l - y * floor(l / y);

        a = const4;

        e = const5 + const6 * T + const7 * T2;
        i = (const8 + const9 * T + const10 * T2) / r2d;
        m = (const11 + const12 * T + const13 * T2) / r2d;
        m = m - y * floor(m / y);
        longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;

        // --------------- Anomalie moyenne

        m1 = (102.27938 + 149472.51529 * T + 0.000007 * T2) / r2d;
        m2 = (212.60322 + 58517.80387 * T + 0.001286 * T2) / r2d;
        m4 = (319.51913 + 19139.85475 * T + 0.000181 * T2) / r2d;
        m5 = (225.32833 + 3034.69202 * T - 0.000722 * T2) / r2d;
        m6 = (175.46622 + 1221.55147 * T - 0.000502 * T2) / r2d;

        m1 = m1 - y * floor(m1 / y);
        m2 = m2 - y * floor(m2 / y);
        m4 = m4 - y * floor(m4 / y);
        m5 = m5 - y * floor(m5 / y);
        m6 = m6 - y * floor(m6 / y);

        l = l + 0.00077 / r2d * sin(237.24 / r2d + 150.27 / r2d * T);
        m = m + 0.00077 / r2d * sin(237.24 / r2d + 150.27 / r2d * T);

        // --------équation de Kepler

        grand_e = m;
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

        r = a * (1 - e * cos(grand_e));

        r = r + 0.000022501 * cos(2 * m - 2 * m2 - 58.208 / r2d);
        r = r + 0.000019045 * cos(3 * m - 3 * m2 + 92.577 / r2d);
        r = r + 0.000006887 * cos(m5 - m2 - 118.09 / r2d);
        r = r + 0.000005172 * cos(m - m2 - 29.11 / r2d);
        r = r + 0.00000362 * cos(5 * m - 4 * m2 - 104.208 / r2d);
        r = r + 0.000003283 * cos(4 * m - 4 * m2 + 63.513 / r2d);
        r = r + 0.000003074 * cos(2 * m5 - 2 * m2 - 55.167 / r2d);

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

        l = l + 0.00313 / r2d * cos(2 * m - 2 * m2 - 148.225 / r2d);
        l = l + 0.00198 / r2d * cos(3 * m - 3 * m2 + 2.565 / r2d);
        l = l + 0.00136 / r2d * cos(m - m2 - 119.107 / r2d);
        l = l + 0.00096 / r2d * cos(3 * m - 2 * m2 - 135.912 / r2d);
        l = l + 0.00082 / r2d * cos(m5 - m2 - 208.087 / r2d);

        if(l > 2 * PI)
        {
            l = l - 2 * PI;
        }

        b = asin(sin(u) * sin(i));

        numerateur = r * cos(b) * sin(l - longitude_terre + PI);
        denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;

        l = atan(numerateur / denominateur) + longitude_terre + PI;

        if(l > 2 * PI)
        {
            l = l - 2 * PI;
        }

        if(denominateur < 0)
        {
            l = l + PI;
        }

        diametre = const17;

        // --------conversion rectangulaire/polaire

        xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
        yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
        zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);

        distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));

        // --------élongation

        omega = 259.18 / r2d - 1934.142 / r2d * T;
        l = l - 0.00479 / r2d * sin(omega);

        elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
        if((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI))
        {
            elongation_planete = "Ouest";
        }
        else
        {
            elongation_planete = "Est";
        }

        double elongationVen = elongation;

        String elongation_planeteVen = elongation_planete;

        String commentVen;
        String quandVen;
        if(elongationVen < 10)
        {
            commentVen = "Inobservable";
            quandVen = " ";
        }

        if(elongationVen > 10 && elongationVen < 20)
        {
            commentVen = "Difficilement visible";
            if(elongation_planeteVen == "Ouest")
            {
                quandVen = "peu avant le lever du Soleil";
            }
            if(elongation_planeteVen == "Est")
            {
                quandVen = "au coucher du Soleil";
            }
        }

        if(elongationVen > 20 && elongationVen < 50)
        {
            commentVen = "Visible";
            if(elongation_planeteVen == "Ouest")
            {
                quandVen = "en toute fin de nuit";
            }
            if(elongation_planeteVen == "Est")
            {
                quandVen = "en tout début de soirée";
            }
        }

        // --------convertion longitude et latitude en ascension droite et déclinaison

        l = l - y * floor(l / y);
        l1 = l * r2d;

        d1 = ((l1 - floor(l1)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        beta = asin(r * sin(b) / distance_terre);

        b1 = abs(beta * r2d);
        d1 = ((b1 - floor(b1)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

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

        double DecVen;
        if(declin < 0)
        {
            signe = "-";
            DecVen = -d / r2d;
        }
        else
        {
            signe = "+";
            DecVen = +d / r2d;
        }

        double ascVen = asc;
        double declinVen = declin;

        Utils.log(ascVen);

        Utils.log(declinVen);
        // Magnitude de la planète

        dist = R;                 // rayon vecteur Soleil-Terre
        ray = r;                // rayon vecteur Soleil-planète
        delta = distance_terre;      // distance Terre-planète

        FV = acos((ray * ray + delta * delta - dist * dist) / (2 * ray * delta));
        FV = (FV) * r2d;

        phase = (1 + cos(FV / r2d)) * 50;

        magnitude = -4.34 + 5 * (log(ray * delta)) / log(10) + 0.013 * FV + 4.2E-7 * pow(FV, 3);
        magnitude = ceil(magnitude * 10) / 10;

        // TODO MARS

        // --------------------------------------------- Mars

        const1 = 293.737334;
        const2 = 19141.69551;
        const3 = 0.0003107;
        const4 = 1.52367934191;
        const5 = 0.0933129;
        const6 = 0.000092064;
        const7 = -0.000000077;
        const8 = 1.850333;
        const9 = -0.000675;
        const10 = 0.0000126;
        const11 = 319.51913;
        const12 = 19139.85475;
        const13 = 0.000181;
        const14 = 48.786442;
        const15 = 0.7709917;
        const16 = -0.0000014;
        const17 = 9.368;

        // --------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i),
        // --------longitude noeud ascendant (m)

        l = (const1 + const2 * T + const3 * T2) / r2d;
        l = l - y * floor(l / y);

        a = const4;

        e = const5 + const6 * T + const7 * T2;
        i = (const8 + const9 * T + const10 * T2) / r2d;
        m = (const11 + const12 * T + const13 * T2) / r2d;
        m = m - y * floor(m / y);
        longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;

        // --------------- Anomalie moyenne

        m1 = (102.27938 + 149472.51529 * T + 0.000007 * T2) / r2d;
        m2 = (212.60322 + 58517.80387 * T + 0.001286 * T2) / r2d;
        m4 = (319.51913 + 19139.85475 * T + 0.000181 * T2) / r2d;
        m5 = (225.32833 + 3034.69202 * T - 0.000722 * T2) / r2d;
        m6 = (175.46622 + 1221.55147 * T - 0.000502 * T2) / r2d;

        m1 = m1 - y * floor(m1 / y);
        m2 = m2 - y * floor(m2 / y);
        m4 = m4 - y * floor(m4 / y);
        m5 = m5 - y * floor(m5 / y);
        m6 = m6 - y * floor(m6 / y);

        // --------corrections

        double corr = -0.01133 / r2d * sin(3 * m5 - 8 * m4 + 4 * m) - 0.00933 / r2d * cos(3 * m5 - 8 * m4 + 4 * m);
        l = l + corr;
        m = m + corr;

        // --------équation de Kepler

        grand_e = m;
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

        r = a * (1 - e * cos(grand_e));

        r = r + 0.000053227 * cos(m5 - m4 + 41.1306 / r2d);
        r = r + 0.000050989 * cos(2 * m5 - 2 * m4 - 101.9847 / r2d);
        r = r + 0.000038278 * cos(2 * m5 - m4 - 98.3292 / r2d);
        r = r + 0.000015996 * cos(m - m4 - 55.555 / r2d);
        r = r + 0.000014764 * cos(2 * m - 3 * m4 + 68.622 / r2d);
        r = r + 0.000008966 * cos(m5 - 2 * m4 + 43.615 / r2d);
        r = r + 0.000007914 * cos(3 * m5 - 2 * m4 - 139.737 / r2d);
        r = r + 0.000007004 * cos(2 * m5 - 3 * m4 - 102.888 / r2d);
        r = r + 0.00000662 * cos(m - 2 * m4 + 113.202 / r2d);
        r = r + 0.00000493 * cos(3 * m5 - 3 * m4 - 76.243 / r2d);
        r = r + 0.000004693 * cos(3 * m - 5 * m4 + 190.603 / r2d);
        r = r + 0.000004571 * cos(2 * m - 4 * m4 + 244.702 / r2d);
        r = r + 0.000004409 * cos(3 * m5 - m4 - 115.828 / r2d);

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

        l = l + 0.00705 / r2d * cos(m5 - m4 - 48.958 / r2d);
        l = l + 0.00607 / r2d * cos(2 * m5 - m4 - 188.35 / r2d);
        l = l + 0.00445 / r2d * cos(2 * m5 - 2 * m4 - 191.897 / r2d);
        l = l + 0.00388 / r2d * cos(m - 2 * m4 + 20.495 / r2d);
        l = l + 0.00238 / r2d * cos(m - m4 + 35.097 / r2d);
        l = l + 0.00204 / r2d * cos(2 * m - 3 * m4 + 158.638 / r2d);
        l = l + 0.00177 / r2d * cos(3 * m4 - m2 - 57.602 / r2d);
        l = l + 0.00136 / r2d * cos(2 * m - 4 * m4 + 154.093 / r2d);
        l = l + 0.00104 / r2d * cos(m5 + 17.618 / r2d);

        if(l > 2 * PI)
        {
            l = l - 2 * PI;
        }

        b = asin(sin(u) * sin(i));

        numerateur = r * cos(b) * sin(l - longitude_terre + PI);
        denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;

        l = atan(numerateur / denominateur) + longitude_terre + PI;

        if(l > 2 * PI)
        {
            l = l - 2 * PI;
        }

        if(denominateur < 0)
        {
            l = l + PI;
        }

        diametre = const17;

        // --------conversion rectangulaire/polaire

        xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
        yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
        zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);

        distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));

        // --------élongation

        omega = 259.18 / r2d - 1934.142 / r2d * T;
        l = l - 0.00479 / r2d * sin(omega);

        elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
        if((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI))
        {
            elongation_planete = "Ouest";
        }
        else
        {
            elongation_planete = "Est";
        }

        double elongationMars = elongation;
        String elongation_planeteMars = elongation_planete;

        String commentMars;
        String quandMars;
        if(elongationMars < 10)
        {
            commentMars = "Inobservable";
            quandMars = " ";
        }

        if(elongationMars > 10 && elongationMars < 20)
        {
            commentMars = "Difficilement observable";
            if(elongation_planeteMars == "Ouest")
            {
                quandMars = "peu avant le lever du Soleil";
            }
            if(elongation_planeteMars == "Est")
            {
                quandMars = "au coucher du Soleil";
            }
        }

        if(elongationMars > 20 && elongationMars < 45)
        {
            commentMars = "Observable";
            if(elongation_planeteMars == "Ouest")
            {
                quandMars = "en toute fin de nuit";
            }
            if(elongation_planeteMars == "Est")
            {
                quandMars = "en tout début de soirée";
            }
        }

        if(elongationMars > 45 && elongationMars < 120)
        {
            commentMars = "Observable";
            if(elongation_planeteMars == "Ouest")
            {
                quandMars = "en seconde partie de nuit";
            }
            if(elongation_planeteMars == "Est")
            {
                quandMars = "en première partie de nuit";
            }
        }

        if(elongationMars > 120 && elongationMars < 140)
        {
            commentMars = "Observable";
            quandMars = "pratiquement toute la nuit";
        }
        if(elongationMars > 140 && elongationMars < 180)
        {
            commentMars = "Observable";
            quandMars = "toute la nuit";
        }

        // --------convertion longitude et latitude en ascension droite et déclinaison

        l = l - y * floor(l / y);
        l1 = l * r2d;

        d1 = ((l1 - floor(l1)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        beta = asin(r * sin(b) / distance_terre);

        b1 = abs(beta * r2d);
        d1 = ((b1 - floor(b1)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

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

        double ADMars = asc * 15 / r2d;

        d = abs(declin);
        d1 = ((d - floor(d)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        double DecMars;
        if(declin < 0)
        {
            signe = "-";
            DecMars = -d / r2d;
        }
        else
        {
            signe = "+";
            DecMars = +d / r2d;
        }

        double ascMars = asc;
        double declinMars = declin;

        // Magnitude de la planète

        dist = R; // rayon vecteur Soleil-Terre
        ray = r; // rayon vecteur Soleil-planète
        delta = distance_terre; // distance Terre-planète

        FV = acos((ray * ray + delta * delta - dist * dist) / (2 * ray * delta));
        FV = (FV) * r2d;

        phase = (1 + cos(FV / r2d)) * 50;

        magnitude = -1.51 + 5 * (log(ray * delta)) / log(10) + 0.016 * FV;
        magnitude = ceil(magnitude * 10) / 10;

        // TODO JUPITER

        //
        // //--------------------------------------------- Jupiter
        //
        // const1 = 238.049257;
        // const2 = 3036.301986;
        // const3 = 0.0003347;
        // const4 = 5.20260300002;
        // const5 = 0.04833475;
        // const6 = 0.00016418;
        // const7 = -0.0000004676;
        // const8 = 1.308736;
        // const9 = -0.0056961;
        // const10 = 0.0000039;
        // const11 = 225.32833;
        // const12 = 3034.69202;
        // const13 = -0.000722;
        // const14 = 99.443414;
        // const15 = 1.01053;
        // const16 = 0.00035222;
        // const17 = 197.146;
        //
        // //--------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i),
        // //--------longitude noeud ascendant (m)
        //
        // l = (const1 + const2 * T + const3 * T2) / r2d;
        // l = l - y * floor(l / y);
        //
        // a = const4;
        //
        // e = const5 + const6 * T + const7 * T2;
        // i = (const8 + const9 * T + const10 * T2) / r2d;
        // m = (const11 + const12 * T + const13 * T2) / r2d;
        // m = m - y * floor(m / y);
        // longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;
        //
        // //--------------- Anomalie moyenne
        //
        // m1 = (102.27938 + 149472.51529 * T + 0.000007 * T2) / r2d;
        // m2 = (212.60322 + 58517.80387 * T + 0.001286 * T2) / r2d;
        // m4 = (319.51913 + 19139.85475 * T + 0.000181 * T2) / r2d;
        // m5 = (225.32833 + 3034.69202 * T - 0.000722 * T2) / r2d;
        // m6 = (175.46622 + 1221.55147 * T - 0.000502 * T2) / r2d;
        //
        // m1= m1 - y * floor(m1 / y);
        // m2= m2 - y * floor(m2 / y);
        // m4 = m4 - y * floor(m4 / y);
        // m5 = m5 - y * floor(m5 / y);
        // m6 = m6 - y * floor(m6 / y);
        //
        // //--------Termes périodiques
        //
        // u = T / 5 + 0.1;
        // p = (237.475 + 3034.9061 * T) / r2d;
        // q = (265.916 + 1222.1139 * T) / r2d;
        // v = 5 * q - 2 * p;
        // dzeta = q - p;
        //
        // //--------perturbations dans la longitude moyenne
        //
        // granda = (0.3314 - 0.0103 * u - 0.0047 * u * u) * sin(v);
        // granda = granda + (0.0032 - 0.0644 * u + 0.0021 * u * u) * cos(v);
        // granda = granda + 0.0136 * sin(dzeta) + 0.0185 * sin(2 * dzeta) + 0.0067 * sin(3 * dzeta);
        // granda = granda + (0.0073 * sin(dzeta) + 0.0064 * sin(2 * dzeta) - 0.0338 * cos(dzeta)) * sin(q);
        // granda = granda - (0.0357 * sin(dzeta) + 0.0063 * cos(dzeta) + 0.0067 * cos(2 * dzeta)) * cos(q);
        //
        // //--------perturbations dans l'excentricité
        //
        // grandb = (361 + 13 * u) * sin(v) + (129 - 58 * u) * cos(v);
        // grandb = grandb + (128 * cos(dzeta) - 676 * sin(dzeta) - 111 * sin(2 * dzeta)) * sin(q);
        // grandb = grandb + (146 * sin(dzeta) - 82 + 607 * cos(dzeta) + 99 * cos(2 * dzeta) + 51 * cos(3 * dzeta)) * cos(q);
        // grandb = grandb - (96 * sin(dzeta) + 100 * cos(dzeta)) * sin(2 * q) - (96 * sin(dzeta) - 102 * cos(dzeta)) * cos(2 * q);
        //
        // //--------perturbations dans la longitude du périhélie
        //
        // grandc = (0.0072 - 0.0031 * u) * sin(v) - 0.0204 * cos(v);
        // grandc = grandc + (0.0073 * sin(dzeta) + 0.034 * cos(dzeta) + 0.0056 * cos(2 * dzeta)) * sin(q);
        // grandc = grandc + (0.0378 * sin(dzeta) + 0.0062 * sin(2 * dzeta) - 0.0066 * cos(dzeta)) * cos(q);
        // grandc = grandc - 0.0054 * sin(dzeta) * sin(2 * q) + 0.0055 * cos(dzeta) * cos(2 * q);
        //
        // //--------perturbations dans le demi-grand axe
        //
        // grandd = -263 * cos(v) + 205 * cos(dzeta) + 693 * cos(2 * dzeta) + 312 * cos(3 * dzeta) + 299 * sin(dzeta) * sin(q) + (204 * sin(2 * dzeta) - 337 * cos(dzeta)) * cos(q);
        //
        // //--------corrections
        //
        // l = l + granda / r2d;
        // m = m + granda / r2d - grandc / r2d / e;
        // e = e + grandb / 1000000;
        // a = a + grandd / 1000000;
        //
        // //--------équation de Kepler
        //
        // grand_e = m;
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        //
        // //--------anomalie vraie
        //
        // v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(grand_e / 2));
        // if (v < 0) {v = v + 2 * PI;}
        //
        // //--------rayon vecteur
        //
        // r = a * (1 - e * cos(grand_e));
        //
        // //--------argument de latitude
        //
        // u = l + v - m - longitude_noeud;
        // u = u - y * floor(u / y);
        //
        // if (cos(u) != 0) {d = atan(cos(i) * tan(u));
        // if (cos(u) < 0) {d = d + PI;}
        // }
        // else {d = u;}
        //
        // //--------longitude écliptique
        //
        // l = d + longitude_noeud;
        //
        // if (l > 2 * PI) {l = l - 2 * PI;}
        //
        // b = asin(sin(u) * sin(i));
        //
        // numerateur = r * cos(b) * sin(l - longitude_terre + PI);
        // denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;
        //
        // l = atan(numerateur / denominateur) + longitude_terre + PI;
        //
        // if (l > 2 * PI) {l = l - 2 * PI;}
        //
        // if (denominateur < 0) {l = l + PI;}
        //
        // diametre = const17;
        //
        // //--------conversion rectangulaire/polaire
        //
        // xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
        // yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
        // zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);
        //
        // distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));
        //
        // document.calc.diametrejup.value = floor(diametre / distance_terre * 100 + 0.5) / 100 + "''";
        //
        // //--------élongation
        //
        // omega = 259.18 / r2d - 1934.142 / r2d * T;
        // l = l - 0.00479 / r2d * sin(omega);
        //
        // elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
        // if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI))
        // {elongation_planete = "Ouest";}
        // else {elongation_planete = "Est";}
        //
        // document.calc.elongatjup.value = floor(elongation * 10 + 0.5) / 10 + "° " + elongation_planete;
        //
        // elongationJup = elongation
        // elongation_planeteJup =elongation_planete
        //
        // if (elongationJup < 20) {commentJup = "Inobservable"; quandJup = " "}
        //
        // if (elongationJup > 20 && elongationJup < 45){
        // commentJup = "Observable"
        // if (elongation_planeteJup == "Ouest") {quandJup = "en toute fin de nuit"}
        // if (elongation_planeteJup == "Est") {quandJup = "en tout début de soirée"}}
        //
        // if (elongationJup > 45 && elongationJup < 120){
        // commentJup = "Observable"
        // if (elongation_planeteJup == "Ouest") {quandJup = "en seconde partie de nuit"}
        // if (elongation_planeteJup == "Est") {quandJup = "en première partie de nuit"}}
        //
        // if (elongationJup > 120 && elongationJup < 140) {commentJup = "Observable"; quandJup = "pratiquement toute la nuit"}
        // if (elongationJup > 140 && elongationJup < 180) {commentJup = "Observable"; quandJup = "toute la nuit"}
        //
        // document.calc.visibjup.value = commentJup + " " + quandJup
        //
        // //--------convertion longitude et latitude en ascension droite et déclinaison
        //
        // l = l -y * floor(l /y);
        // l1 = l * r2d;
        //
        // d1 = ((l1 - floor(l1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.longjup.value = floor(l1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        //
        // beta = asin(r * sin(b) / distance_terre);
        //
        // b1 = abs(beta * r2d);
        // d1 = ((b1 - floor(b1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // if (beta < 0) {signe = "-";}
        // else {signe = "+";}
        //
        // document.calc.latjup.value = signe + floor(b1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // document.calc.rayonjup.value = floor(r * 100000 + 0.5) / 100000 + " UA"
        // document.calc.distancejup.value = floor(distance_terre * 100000 + 0.5) / 100000 + " UA"
        //
        // asc = atan((cos(obliquite) * sin(l) - tan(beta) * sin(obliquite)) / cos(l));
        // if (asc < 0) {asc = asc + 2 * PI;}
        // if (cos(l) < 0) {asc = asc + PI;}
        // if (asc > 2 * PI) {asc = asc - 2 * PI;}
        // declin = r2d * asin(sin(beta) * cos(obliquite) + cos(beta) * sin(obliquite) * sin(l));
        //
        // asc = asc * r2d / 15;
        //
        // d1 = ((asc - floor(asc)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.alphajup.value = floor(asc) + "h" + zero(floor(d1)) + "m" + zero(floor(d2 * 100 + 0.5) / 100) + "s";
        // ADJup = asc * 15 / r2d
        //
        // d = abs(declin);
        // d1 = ((d - floor(d)) * 60);
        // d2= ((d1 - floor(d1)) * 60);
        //
        // if (declin < 0) {signe = "-"; DecJup = - d / r2d}
        // else {signe = "+"; DecJup = + d / r2d}
        //
        // document.calc.deltajup.value = signe + floor(d) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // ascJup = asc
        // declinJup = declin
        //
        // //Magnitude de la planète
        //
        // dist = R // rayon vecteur Soleil-Terre
        // ray = r // rayon vecteur Soleil-planète
        // delta = distance_terre // distance Terre-planète
        //
        // FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) )
        // FV = (FV) * r2d
        //
        // phase = (1 + cos(FV / r2d)) * 50
        //
        // magnitude = - 9.25 + 5 * (log(ray * delta))/ log(10) + 0.014 * FV
        // magnitude = ceil(magnitude * 10) / 10
        //
        // document.calc.magjup.value = magnitude;
        // document.calc.phasejup.value = floor(phase * 10) / 10 + "%";
        //
        // //--------------------------------------------- Saturne
        //
        // const1 = 266.564377;
        // const2 = 1223.509884;
        // const3 = 0.0003245;
        // const4 = 9.55491173474;
        // const5 = 0.05589232;
        // const6 = -0.0003455;
        // const7 = -0.000000728;
        // const8 = 2.492519;
        // const9 = -0.0039189;
        // const10 = -0.00001549;
        // const11 = 175.46622;
        // const12 = 1221.55147;
        // const13 = -0.000502;
        // const14 = 112.790414;
        // const15 = 0.8731951;
        // const16 = -0.00015218;
        // const17 = 166.194;
        //
        // //--------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i),
        // //--------longitude noeud ascendant (m)
        //
        // l = (const1 + const2 * T + const3 * T2) / r2d;
        // l = l - y * floor(l / y);
        //
        // a = const4;
        //
        // e = const5 + const6 * T + const7 * T2;
        // i = (const8 + const9 * T + const10 * T2) / r2d;
        // m = (const11 + const12 * T + const13 * T2) / r2d;
        // m = m - y * floor(m / y);
        // longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;
        //
        // //--------------- Anomalie moyenne
        //
        // m1 = (102.27938 + 149472.51529 * T + 0.000007 * T2) / r2d;
        // m2 = (212.60322 + 58517.80387 * T + 0.001286 * T2) / r2d;
        // m4 = (319.51913 + 19139.85475 * T + 0.000181 * T2) / r2d;
        // m5 = (225.32833 + 3034.69202 * T - 0.000722 * T2) / r2d;
        // m6 = (175.46622 + 1221.55147 * T - 0.000502 * T2) / r2d;
        //
        // m1= m1 - y * floor(m1 / y);
        // m2= m2 - y * floor(m2 / y);
        // m4 = m4 - y * floor(m4 / y);
        // m5 = m5 - y * floor(m5 / y);
        // m6 = m6 - y * floor(m6 / y);
        //
        // //--------Termes périodiques
        //
        // u = T / 5 + 0.1;
        // p = (237.475 + 3034.9061 * T) / r2d;
        // q = (265.916 + 1222.1139 * T) / r2d;
        // v = 5 * q - 2 * p;
        // dzeta = q - p;
        //
        // //--------perturbations dans la longitude moyenne
        //
        // granda = (-0.8142 + 0.0181 * u + 0.0167 * u * u) * sin(v);
        // granda = granda + (-0.0105 + 0.1609 * u - 0.0041 * u * u) * cos(v);
        // granda = granda - 0.1488 * sin(dzeta) - 0.0408 * sin(2 * dzeta) - 0.0152 * sin(3 * dzeta);
        // granda = granda + (0.0089 * sin(dzeta) - 0.0165 * sin(2 * dzeta)) * sin(q);
        // granda = granda + (0.0813 * cos(dzeta) + 0.015 * cos(2 * dzeta)) * sin(q);
        // granda = granda + (0.0856 * sin(dzeta) + 0.0253 * cos(dzeta) + 0.0144 * cos(2 * dzeta)) * cos(q);
        // granda = granda + 0.0092 * sin(2 * dzeta) * sin(2 * q);
        //
        // //--------perturbations dans l'excentricité
        //
        // grandb = (-793 + 255 * u) * sin(v) + (1338 + 123 * u) * cos(v);
        // grandb = grandb + 1241 * sin(q) + (39 - 62 * u) * sin(dzeta) * sin(q);
        // grandb = grandb + (2660 * cos(dzeta) - 469 * cos(2 * dzeta) - 187 * cos(3 * dzeta) - 82 * cos(4 * dzeta)) * sin(q);
        // grandb = grandb - (1270 * sin(dzeta) + 420 * sin(2 * dzeta) + 150 * sin(3 * dzeta)) * cos(q);
        // grandb = grandb - 62 * sin(4 * dzeta) * cos(q);
        // grandb = grandb + (221 * sin(dzeta) - 221 * sin(2 * dzeta) - 57 * sin(3 * dzeta)) * sin(2 * q);
        // grandb = grandb - (278 * cos(dzeta) - 202 * cos(2 * dzeta)) * sin(2 * q);
        // grandb = grandb - (284 * sin(dzeta) + 159 * cos(dzeta)) * cos(2 * q);
        // grandb = grandb + (216 * cos(2 * dzeta) + 56 * cos(3 * dzeta)) * cos(2 * q);
        //
        // //--------perturbations dans la longitude du périhélie
        //
        // grandc = (0.0771 + 0.0072 * u) * sin(v);
        // grandc = grandc + (0.0458 - 0.0148 * u) * cos(v);
        // grandc = grandc - (0.0758 * sin(dzeta) + 0.0248 * sin(2 * dzeta) + 0.0086 * sin(3 * dzeta)) * sin(q);
        // grandc = grandc - (0.0726 + 0.1504 * cos(dzeta) - 0.0269 * cos(2 * dzeta) - 0.0101 * cos(3 * dzeta)) * cos(q);
        // grandc = grandc - (0.0136 * sin(dzeta) - 0.0136 * cos(2 * dzeta)) * sin(2 * q);
        // grandc = grandc - (0.0137 * sin(dzeta) - 0.012 * sin(2 * dzeta)) * cos(2 * q);
        // grandc = grandc + (0.0149 * cos(dzeta) - 0.0131 * cos(2 * dzeta)) * cos(2 * q);
        //
        // //--------perturbations dans le demi-grand axe
        //
        // grandd = 2933 * cos(v) + 33629 * cos(dzeta) - 3081 * cos(2 * dzeta) - 1423 * cos(3 * dzeta) - 671 * cos(4 * dzeta) + (1098 - 2812 * sin(dzeta) + 688 * sin(2 * dzeta)) * sin(q);
        // grandd = grandd + (2138 * cos(dzeta) - 999 * cos(2 * dzeta) - 642 * cos(3 * dzeta)) * sin(q) - 890 * cos(q) + (2206 * sin(dzeta) - 1590 * sin(2 * dzeta) - 647 * sin(3 * dzeta)) * cos(q) +
        // (2885 * cos(dzeta) + 2172 * cos(2 * dzeta)) * cos(q) - 778 * cos(dzeta) * sin(2 * q) - 856 * sin(dzeta) * cos(2 * q);
        //
        // //--------corrections
        //
        // l = l + granda / r2d;
        // m = m + granda / r2d - grandc / r2d / e;
        // e = e + grandb / 1000000;
        // a = a + grandd / 1000000;
        //
        // //--------équation de Kepler
        //
        // grand_e = m;
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        //
        // //--------anomalie vraie
        //
        // v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(grand_e / 2));
        // if (v < 0) {v = v + 2 * PI;}
        //
        // //--------rayon vecteur
        //
        // r = a * (1 - e * cos(grand_e));
        //
        // //--------argument de latitude
        //
        // u = l + v - m - longitude_noeud;
        // u = u - y * floor(u / y);
        //
        // if (cos(u) != 0) {d = atan(cos(i) * tan(u));
        // if (cos(u) < 0) {d = d + PI;}
        // }
        // else {d = u;}
        //
        // //--------longitude écliptique
        //
        // l = d + longitude_noeud;
        //
        // if (l > 2 * PI) {l = l - 2 * PI;}
        //
        // b = asin(sin(u) * sin(i));
        //
        // corr = 0.000747 * cos(dzeta) * sin(q) + 0.001069 * cos(dzeta) * cos(q);
        // corr = corr + 0.002108 * sin(2 * dzeta) * sin(2 * q) + 0.001261 * cos(2 * dzeta) * sin(2 * q);
        // corr = corr + 0.001236 * sin(2 * dzeta) * cos(2 * q) - 0.002075 * cos(2 * dzeta) * cos(2 * q);
        // b = b + corr / r2d;
        //
        // numerateur = r * cos(b) * sin(l - longitude_terre + PI);
        // denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;
        //
        // l = atan(numerateur / denominateur) + longitude_terre + PI;
        //
        // if (l > 2 * PI) {l = l - 2 * PI;}
        //
        // if (denominateur < 0) {l = l + PI;}
        //
        // diametre = const17;
        //
        // //--------conversion rectangulaire/polaire
        //
        // xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
        // yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
        // zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);
        //
        // distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));
        //
        // document.calc.diametresat.value = floor(diametre / distance_terre * 100 + 0.5) / 100 + "''";
        //
        // //--------élongation
        //
        // omega = 259.18 / r2d - 1934.142 / r2d * T;
        // l = l - 0.00479 / r2d * sin(omega);
        //
        // elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
        // if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI))
        // {elongation_planete = "Ouest";}
        // else {elongation_planete = "Est";}
        //
        // document.calc.elongatsat.value = floor(elongation * 10 + 0.5) / 10 + "° " + elongation_planete;
        //
        // elongationSat = elongation
        // elongation_planeteSat =elongation_planete
        //
        // if (elongationSat < 20) {commentSat = "Inobservable"; quandSat = " "}
        //
        // if (elongationSat > 20 && elongationSat < 45){
        // commentSat = "Observable"
        // if (elongation_planeteSat == "Ouest") {quandSat = "en toute fin de nuit"}
        // if (elongation_planeteSat == "Est") {quandSat = "en tout début de soirée"}}
        //
        // if (elongationSat > 45 && elongationSat < 120){
        // commentSat = "Observable"
        // if (elongation_planeteSat == "Ouest") {quandSat = "en seconde partie de nuit"}
        // if (elongation_planeteSat == "Est") {quandSat = "en première partie de nuit"}}
        //
        // if (elongationSat > 120 && elongationSat < 140) {commentSat = "Observable"; quandSat = "pratiquement toute la nuit"}
        // if (elongationSat > 140 && elongationSat < 180) {commentSat = "Observable"; quandSat = "toute la nuit"}
        //
        // document.calc.visibsat.value = commentSat + " " + quandSat
        //
        // //--------convertion longitude et latitude en ascension droite et déclinaison
        //
        // l = l -y * floor(l /y);
        // l1 = l * r2d;
        //
        // d1 = ((l1 - floor(l1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.longsat.value = floor(l1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        //
        // beta = asin(r * sin(b) / distance_terre);
        //
        // b1 = abs(beta * r2d);
        // d1 = ((b1 - floor(b1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // if (beta < 0) {signe = "-";}
        // else {signe = "+";}
        //
        // document.calc.latsat.value = signe + floor(b1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // document.calc.rayonsat.value = floor(r * 100000 + 0.5) / 100000 + " UA"
        // document.calc.distancesat.value = floor(distance_terre * 100000 + 0.5) / 100000 + " UA"
        //
        // asc = atan((cos(obliquite) * sin(l) - tan(beta) * sin(obliquite)) / cos(l));
        // if (asc < 0) {asc = asc + 2 * PI;}
        // if (cos(l) < 0) {asc = asc + PI;}
        // if (asc > 2 * PI) {asc = asc - 2 * PI;}
        //
        // declin = r2d * asin(sin(beta) * cos(obliquite) + cos(beta) * sin(obliquite) * sin(l));
        //
        // asc = asc * r2d / 15;
        //
        // d1 = ((asc - floor(asc)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.alphasat.value = floor(asc) + "h" + zero(floor(d1)) + "m" + zero(floor(d2 * 100 + 0.5) / 100) + "s";
        // ADSat = asc * 15 / r2d
        //
        // d = abs(declin);
        // d1 = ((d - floor(d)) * 60);
        // d2= ((d1 - floor(d1)) * 60);
        //
        // if (declin < 0) {signe = "-"; DecSat = - d / r2d}
        // else {signe = "+"; DecSat = + d / r2d}
        //
        // document.calc.deltasat.value = signe + floor(d) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // ascSat = asc
        // declinSat = declin
        //
        // //Magnitude de la planète
        //
        // dist = R // rayon vecteur Soleil-Terre
        // ray = r // rayon vecteur Soleil-planète
        // delta = distance_terre // distance Terre-planète
        //
        // FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) )
        // FV = (FV) * r2d
        //
        // phase = (1 + cos(FV / r2d)) * 50
        //
        // magnitude = - 9.0 + 5 * (log(ray * delta))/ log(10) + 0.044 * FV
        // magnitude = floor(magnitude * 100) / 100
        //
        // los = l1 //geocentric ecliptic longitude
        // las = b1 //geocentric ecliptic latitude
        //
        // ir = 28.06 / r2d
        // Nr = (169.51 / r2d) + (3.82E-5 / r2d) * T
        // B = asin( sin(las) * cos(ir) - cos(las) * sin(ir) * sin(los - Nr) )
        //
        // ring_magn = -2.6 * sin(abs(B)) + 1.2 * pow(sin(B),2)
        // ring_magn = floor(ring_magn * 100) / 100
        //
        // document.calc.magsat.value = ceil((magnitude + ring_magn) * 10) / 10;
        // document.calc.phasesat.value = floor(phase * 10) / 10 + "%";
        //
        // //--------------------------------------------- Uranus
        //
        // const1 = 244.19747;
        // const2 = 429.863546;
        // const3 = 0.000316;
        // const4 = 19.21844609894;
        // const5 = 0.0463444;
        // const6 = -0.00002658;
        // const7 = 0.000000077;
        // const8 = 0.772464;
        // const9 = 0.0006253;
        // const10 = 0.0000395;
        // const11 = 72.648778;
        // const12 = 428.3791132;
        // const13 = 0.0000788;
        // const14 = 73.477111;
        // const15 = 0.4986678;
        // const16 = 0.0013117;
        // const17 = 70.481;
        //
        // //--------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i),
        // //--------longitude noeud ascendant (m)
        //
        // l = (const1 + const2 * T + const3 * T2) / r2d;
        // l = l - y * floor(l / y);
        //
        // a = const4;
        //
        // e = const5 + const6 * T + const7 * T2;
        // i = (const8 + const9 * T + const10 * T2) / r2d;
        // m = (const11 + const12 * T + const13 * T2) / r2d;
        // m = m - y * floor(m / y);
        // longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;
        //
        // //--------------- Anomalie moyenne
        //
        // m1 = (102.27938 + 149472.51529 * T + 0.000007 * T2) / r2d;
        // m2 = (212.60322 + 58517.80387 * T + 0.001286 * T2) / r2d;
        // m4 = (319.51913 + 19139.85475 * T + 0.000181 * T2) / r2d;
        // m5 = (225.32833 + 3034.69202 * T - 0.000722 * T2) / r2d;
        // m6 = (175.46622 + 1221.55147 * T - 0.000502 * T2) / r2d;
        //
        // m1= m1 - y * floor(m1 / y);
        // m2= m2 - y * floor(m2 / y);
        // m4 = m4 - y * floor(m4 / y);
        // m5 = m5 - y * floor(m5 / y);
        // m6 = m6 - y * floor(m6 / y);
        //
        // //--------Termes périodiques
        //
        // u = T / 5 + 0.1;
        // p = (237.475 + 3034.9061 * T) / r2d;
        // q = (265.916 + 1222.1139 * T) / r2d;
        // g = (83.76922 + 218.4901 * T) / r2d;
        // h = (284.02 + 8.51 * T) / r2d;
        // s = (243.52 + 428.47 * T) / r2d;
        // w = 2 * p - 6 * q + 3 * s;
        // dzeta = s - p;
        // eta = s - q;
        // theta = (200.25 - 209.98 * T) / r2d;
        //
        // //--------perturbations
        //
        // granda = (0.864319 - 0.001583 * u) * sin(h) + 0.036017 * sin(2 * h) + (0.082222 - 0.006833 * u) * cos(h) - 0.003019 * cos(2 * h) + 0.008122 * sin(w);
        // grandb = 2098 * cos(h) - 335 * sin(h) + 131 * cos(2 * h);
        // grandc = 0.120303 * sin(h) + (0.019472 - 0.000947 * u) * cos(h) + 0.006197 * sin(2 * h);
        // grandd = -3825 * cos(h);
        //
        // //--------corrections
        //
        // l = l + granda / r2d;
        // m = m + granda / r2d - grandc / r2d / e;
        // e = e + grandb / 1000000;
        // a = a + grandd / 1000000;
        //
        // //--------équation de Kepler
        //
        // grand_e = m;
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        //
        // //--------anomalie vraie
        //
        // v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(grand_e / 2));
        // if (v < 0) {v = v + 2 * PI;}
        //
        // //--------rayon vecteur
        //
        // r = a * (1 - e * cos(grand_e));
        //
        // r = r - 0.025948 + 0.004985 * cos(dzeta) - 0.00123 * cos(s) + 0.003354 * cos(eta);
        // r = r + (0.005795 * cos(s) - 0.001165 * sin(s) + 0.001388 * cos(2 * s)) * sin(eta);
        // r = r + (0.001351 * cos(s) + 0.005702 * sin(s) + 0.00138 * sin(2 * s)) * cos(eta);
        // r = r + 0.000904 * cos(2 * theta) + 0.000894 * (cos(theta) - cos(3 * theta));
        //
        // //--------argument de latitude
        //
        // u = l + v - m - longitude_noeud;
        // u = u - y * floor(u / y);
        //
        // if (cos(u) != 0) {d = atan(cos(i) * tan(u));
        // if (cos(u) < 0) {d = d + PI;}
        // }
        // else {d = u;}
        //
        // //--------longitude écliptique
        //
        // l = d + longitude_noeud;
        //
        // l = l + ((0.010122 - 0.000988 * u) * sin(s + eta) + (-0.038581 + 0.002031 * u - 0.00191 * u * u) * cos(s + eta) + (0.034964 - 0.001038 * u + 0.000868 * u * u) * cos(2 * s + eta) - 0.014808
        // * sin(dzeta) - 0.005794 * sin(eta) + 0.002347 * cos(eta) + 0.009872 * sin(theta) + 0.008803 * sin(2 * theta) - 0.004308 * sin(3 * theta)) / r2d;
        //
        // if (l > 2 * PI) {l = l - 2 * PI;}
        //
        // b = asin(sin(u) * sin(i));
        //
        // corr = (0.000458 * sin(eta) - 0.000642 * cos(eta) - 0.000517 * cos(4 * theta)) * sin(s);
        // corr = corr - (0.000347 * sin(eta) + 0.000853 * cos(eta) + 0.000517 * sin(4 * eta)) * cos(s);
        // corr = corr + 0.000403 * (cos(2 * theta) * sin(2 * s) + sin(2 * theta) * cos(2 * s));
        // b = b + corr / r2d;
        //
        // numerateur = r * cos(b) * sin(l - longitude_terre + PI);
        // denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;
        //
        // l = atan(numerateur / denominateur) + longitude_terre + PI;
        //
        // if (l > 2 * PI) {l = l - 2 * PI;}
        //
        // if (denominateur < 0) {l = l + PI;}
        //
        // diametre = const17;
        //
        // //--------conversion rectangulaire/polaire
        //
        // xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
        // yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
        // zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);
        //
        // distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));
        //
        // document.calc.diametreuran.value = floor(diametre / distance_terre * 100 + 0.5) / 100 + "''";
        //
        // //--------élongation
        //
        // omega = 259.18 / r2d - 1934.142 / r2d * T;
        // l = l - 0.00479 / r2d * sin(omega);
        //
        // elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
        // if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI)) {
        // elongation_planete = "Ouest";
        // }
        // else
        // {
        // elongation_planete = "Est";
        // }
        //
        // document.calc.elongaturan.value = floor(elongation * 10 + 0.5) / 10 + "° "+ elongation_planete;
        //
        // elongationUran = elongation
        // elongation_planeteUran =elongation_planete
        //
        // if (elongationUran < 20) {commentUran = "Inobservable"; quandUran = " "}
        //
        // if (elongationUran > 20 && elongationUran < 45){
        // commentUran = "Observable"
        // if (elongation_planeteUran == "Ouest") {quandUran = "en toute fin de nuit"}
        // if (elongation_planeteUran == "Est") {quandUran = "en tout début de soirée"}}
        //
        // if (elongationUran > 45 && elongationUran < 120){
        // commentUran = "Observable"
        // if (elongation_planeteUran == "Ouest") {quandUran = "en seconde partie de nuit"}
        // if (elongation_planeteUran == "Est") {quandUran = "en première partie de nuit"}}
        //
        // if (elongationUran > 120 && elongationUran < 140) {commentUran = "Observable"; quandUran = "pratiquement toute la nuit"}
        // if (elongationUran > 140 && elongationUran < 180) {commentUran = "Observable"; quandUran = "toute la nuit"}
        //
        // document.calc.visiburan.value = commentUran + " " + quandUran
        //
        // //--------convertion longitude et latitude en ascension droite et déclinaison
        //
        // l = l -y * floor(l /y);
        // l1 = l * r2d;
        //
        // d1 = ((l1 - floor(l1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.longuran.value = floor(l1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        //
        // beta = asin(r * sin(b) / distance_terre);
        //
        // b1 = abs(beta * r2d);
        // d1 = ((b1 - floor(b1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // if (beta < 0) {signe = "-";}
        // else {signe = "+";}
        //
        // document.calc.laturan.value = signe + floor(b1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // document.calc.rayonuran.value = floor(r * 100000 + 0.5) / 100000 + " UA"
        // document.calc.distanceuran.value = floor(distance_terre * 100000 + 0.5) / 100000 + " UA"
        //
        // asc = atan((cos(obliquite) * sin(l) - tan(beta) * sin(obliquite)) / cos(l));
        // if (asc < 0) {asc = asc + 2 * PI;}
        // if (cos(l) < 0) {asc = asc + PI;}
        // if (asc > 2 * PI) {asc = asc - 2 * PI;}
        //
        // declin = r2d * asin(sin(beta) * cos(obliquite) + cos(beta) * sin(obliquite) * sin(l));
        //
        // asc = asc * r2d / 15;
        //
        // d1 = ((asc-floor(asc)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.alphauran.value = floor(asc) + "h" + zero(floor(d1)) + "m" + zero(floor(d2 * 100 + 0.5) / 100) + "s";
        // ADUran = asc * 15 / r2d
        //
        // d = abs(declin);
        // d1 = ((d - floor(d)) * 60);
        // d2= ((d1 - floor(d1)) * 60);
        //
        // if (declin < 0) {signe = "-"; DecUran = - d / r2d}
        // else {signe = "+"; DecUran = + d / r2d}
        //
        // document.calc.deltauran.value = signe + floor(d) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // ascUran = asc
        // declinUran = declin
        //
        // //Magnitude de la planète
        //
        // dist = R // rayon vecteur Soleil-Terre
        // ray = r // rayon vecteur Soleil-planète
        // delta = distance_terre // distance Terre-planète
        //
        // FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) )
        // FV = (FV) * r2d
        //
        // phase = (1 + cos(FV / r2d)) * 50
        //
        // magnitude = - 7.15 + 5 * (log(ray * delta))/ log(10) + 0.001 * FV
        // magnitude = ceil(magnitude * 100)/100
        //
        // document.calc.maguran.value = magnitude;
        // document.calc.phaseuran.value = floor(phase * 10) / 10 + "%";
        //
        // //--------------------------------------------- Neptune
        //
        // const1 = 84.457994;
        // const2 = 219.885914;
        // const3 = 0.0003205;
        // const4 = 30.11038703542;
        // const5 = 0.00899704;
        // const6 = 0.00000633;
        // const7 = -0.000000002;
        // const8 = 1.779242;
        // const9 = -0.0095436;
        // const10 = -0.0000091;
        // const11 = 37.73063;
        // const12 = 218.4613396;
        // const13 = -0.00007032;
        // const14 = 130.681389;
        // const15 = 1.098935;
        // const16 = 0.00024987;
        // const17 = 68.289;
        //
        // //--------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i),
        // //--------longitude noeud ascendant (m)
        //
        // l = (const1 + const2 * T + const3 * T2) / r2d;
        // l = l - y * floor(l / y);
        //
        // a = const4;
        //
        // e = const5 + const6 * T + const7 * T2;
        // i = (const8 + const9 * T + const10 * T2) / r2d;
        // m = (const11 + const12 * T + const13 * T2) / r2d;
        // m = m - y * floor(m / y);
        // longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;
        //
        // //--------------- Anomalie moyenne
        //
        //
        // //--------Termes périodiques
        //
        // u = T / 5 + 0.1;
        // p = (237.475 + 3034.9061 * T) / r2d;
        // q = (265.916 + 1222.1139 * T) / r2d;
        // g = (83.76922 + 218.4901 * T) / r2d;
        // h = (284.02 + 8.51 * T) / r2d;
        // theta = (200.25 - 209.98 * T) / r2d;
        // dzeta = (153.71 + 2816.42 * T) / r2d;
        // eta = (182.15 + 1003.62 * T) / r2d;
        //
        // //--------perturbations
        //
        // granda = (-0.589833 + 0.001089 * u) * sin(h) + (-0.056094 + 0.004658 * u) * cos(h) - 0.024286 * sin(2 * h);
        // grandb = 438.9 * sin(h) + 426.2 * cos(h) + 112.9 * sin(2 * h) + 108.9 * cos(2 * h);
        // grandc = 0.024039 * sin(h) - 0.025303 * cos(h) + 0.006206 * sin(2 * h) - 0.005992 * cos(2 * h);
        // grandd = -817 * sin(h) + 8189 * cos(h) + 781 * cos(2 * h);
        //
        // //--------corrections
        //
        // l = l + granda / r2d;
        // m = m + granda / r2d - grandc / r2d / e;
        // e = e + grandb / 1000000;
        // a = a + grandd / 1000000;
        //
        // //--------équation de Kepler
        //
        // grand_e = m;
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        //
        // //--------anomalie vraie
        //
        // v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(grand_e / 2));
        // if (v < 0) {v = v + 2 * PI;}
        //
        // //--------rayon vecteur
        //
        // r = a * (1 - e * cos(grand_e));
        //
        // r = r - 0.040596 + 0.004992 * cos(dzeta) + 0.002744 * cos(eta) + 0.002044 * cos(theta) + 0.001051 * cos(2 * theta);
        //
        // //--------argument de latitude
        //
        // u = l + v - m - longitude_noeud;
        // u = u - y * floor(u / y);
        //
        // if (cos(u) != 0) {d = atan(cos(i) * tan(u));
        // if (cos(u) < 0) {d = d + PI;}
        // }
        // else {d = u;}
        //
        // //--------longitude écliptique
        //
        // l = d + longitude_noeud;
        //
        // l = l + (0.009556 * sin(dzeta) + 0.005178 * sin(eta)) / r2d;
        //
        // if (l > 2 * PI) {l = l - 2 * PI;}
        //
        // b = asin(sin(u) * sin(i));
        //
        // b = b + (0.000336 * cos(2 * theta) * sin(g) + 0.000364 * sin(2 * theta) * cos(g)) / r2d;
        //
        // numerateur = r * cos(b) * sin(l - longitude_terre + PI);
        // denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;
        //
        // l = atan(numerateur / denominateur) + longitude_terre + PI;
        //
        // if (l > 2 * PI) {l = l - 2 * PI;}
        // if (denominateur < 0) {l = l + PI;}
        //
        // diametre = const17;
        //
        // //--------conversion rectangulaire/polaire
        //
        // xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
        // yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
        // zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);
        //
        // distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));
        //
        // document.calc.diametrenept.value = floor(diametre / distance_terre * 100 + 0.5) / 100 + "''";
        //
        // //--------élongation
        //
        // omega = 259.18 / r2d - 1934.142 / r2d * T;
        // l = l - 0.00479 / r2d * sin(omega);
        //
        // elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
        // if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI))
        // {elongation_planete = "Ouest";}
        // else {elongation_planete = "Est";}
        //
        // document.calc.elongatnept.value = floor(elongation * 10 + 0.5) / 10 + "° " + elongation_planete;
        //
        // elongationNept = elongation
        // elongation_planeteNept =elongation_planete
        //
        // if (elongationNept < 20) {commentNept = "Inobservable"; quandNept = " "}
        //
        // if (elongationNept > 20 && elongationNept < 45){
        // commentNept = "Observable"
        // if (elongation_planeteNept == "Ouest") {quandNept = "en toute fin de nuit"}
        // if (elongation_planeteNept == "Est") {quandNept = "en tout début de soirée"}}
        //
        // if (elongationNept > 45 && elongationNept < 120){
        // commentNept = "Observable"
        // if (elongation_planeteNept == "Ouest") {quandNept = "en seconde partie de nuit"}
        // if (elongation_planeteNept == "Est") {quandNept = "en première partie de nuit"}}
        //
        // if (elongationNept > 120 && elongationNept < 140) {commentNept = "Observable"; quandNept = "pratiquement toute la nuit"}
        // if (elongationNept > 140 && elongationNept < 180) {commentNept = "Observable"; quandNept = "toute la nuit"}
        //
        // document.calc.visibnept.value = commentNept + " " + quandNept
        //
        // //--------convertion longitude et latitude en ascension droite et déclinaison
        //
        // l = l -y * floor(l /y);
        // l1 = l * r2d;
        //
        // d1 = ((l1 - floor(l1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.longnept.value = floor(l1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        //
        // beta = asin(r * sin(b) / distance_terre);
        //
        // b1 = abs(beta * r2d);
        // d1 = ((b1 - floor(b1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // if (beta < 0) {signe = "-";}
        // else {signe = "+";}
        //
        // document.calc.latnept.value = signe + floor(b1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // document.calc.rayonnept.value = floor(r * 100000 + 0.5) / 100000 + " UA"
        // document.calc.distancenept.value = floor(distance_terre * 100000 + 0.5) / 100000 + " UA"
        //
        // asc = atan((cos(obliquite) * sin(l) - tan(beta) * sin(obliquite)) / cos(l));
        // if (asc < 0) {asc = asc + 2 * PI;}
        // if (cos(l) < 0) {asc = asc + PI;}
        // if (asc > 2 * PI) {asc = asc - 2 * PI;}
        //
        // declin = r2d * asin(sin(beta) * cos(obliquite) + cos(beta) * sin(obliquite) * sin(l));
        //
        // asc = asc * r2d / 15;
        //
        // d1 = ((asc - floor(asc)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.alphanept.value = floor(asc) + "h" + zero(floor(d1)) + "m" + zero(floor(d2 * 100 + 0.5) / 100) + "s";
        // ADNept = asc * 15 / r2d
        //
        // d = abs(declin);
        // d1 = ((d - floor(d)) * 60);
        // d2= ((d1 - floor(d1)) * 60);
        //
        // if (declin < 0) {signe = "-"; DecNept = - d / r2d}
        // else {signe = "+"; DecNept = + d / r2d}
        //
        // document.calc.deltanept.value = signe + floor(d) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // ascNept = asc
        // declinNept = declin
        //
        // //Magnitude de la planète
        //
        // dist = R // rayon vecteur Soleil-Terre
        // ray = r // rayon vecteur Soleil-planète
        // delta = distance_terre // distance Terre-planète
        //
        // FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) )
        // FV = (FV) * r2d
        //
        // phase = (1 + cos(FV / r2d)) * 50
        //
        // magnitude = - 6.90 + 5 * (log(ray * delta))/ log(10) + 0.001 * FV
        // magnitude = ceil(magnitude * 100)/100
        //
        // document.calc.magnept.value = magnitude;
        // document.calc.phasenept.value = floor(phase * 10) / 10 + "%";
        //
        // //--------------------------------------------- Pluton
        //
        // const17 = 3.200;
        //
        // //--------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i),
        // //--------longitude noeud ascendant (m)
        //
        // l = mod2pi((238.92881 + (522747.90 * T_2000 / 3600)) / r2d); //longitude moyenne
        // l = l - y * floor(l / y);
        // a = 39.48168677 - (0.00076912 * T_2000); // demi-grand axe
        // e = 0.24880766 + (0.00006465 * T_2000); // excentricité
        // i = (17.14175 + (11.07 * T_2000 / 3600)) / r2d; // inclinaison
        // ap = (224.06676 - (132.25 * T_2000 / 3600)) / r2d; // argument du périhélie
        // Om = (110.30347 - (37.33 * T_2000 / 3600)) / r2d; // longitude du noeud ascendant
        //
        // m = (l - ap) // longitude du périhélie
        //
        // m = m - y * floor(m / y);
        //
        // //--------équation de Kepler
        //
        // grand_e = m;
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        // grand_e = m + e * sin(grand_e);
        //
        // //--------anomalie vraie
        //
        // v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(grand_e / 2));
        // if (v < 0) {v = v + 2 * PI;}
        //
        // //--------rayon vecteur
        //
        // r = a * (1 - e * cos(grand_e));
        //
        // //--------argument de latitude
        //
        // u = l + v - m - Om;
        // u = u - y * floor(u / y);
        // if (cos(u) != 0) {
        // d = atan(cos(i) * tan(u));
        // if (cos(u) < 0) {
        // d = d + PI;
        // }
        // }
        // else {
        // d = u;
        // }
        //
        // //--------longitude écliptique
        //
        // l = d + Om;
        // if (l > 2 * PI)
        // {
        // l = l - 2 * PI;
        // }
        // b = asin(sin(u) * sin(i));
        // numerateur = r * cos(b) * sin(l - longitude_terre + PI);
        // denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;
        // l = atan(numerateur / denominateur) + longitude_terre + PI;
        // if (l > 2 * PI)
        // {l = l - 2 * PI;}
        // if (denominateur < 0) {l = l + PI;}
        //
        // diametre = const17;
        //
        // //--------conversion rectangulaire/polaire
        //
        // xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
        // yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
        // zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);
        //
        // distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));
        //
        // document.calc.diametrepluton.value = floor(diametre / distance_terre * 100 + 0.5) / 100 + "''";
        //
        // //--------élongation
        //
        // omega = 259.18 / r2d - 1934.142 / r2d * T_2000;
        // l = l - 0.00479 / r2d * sin(omega);
        // elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
        // if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI))
        // {elongation_planete = "Ouest";}
        // else {elongation_planete = "Est";}
        //
        // document.calc.elongatpluton.value = floor(elongation * 10 + 0.5) / 10 + "° " + elongation_planete;
        //
        // elongationPluton = elongation
        // elongation_planetePluton =elongation_planete
        //
        // if (elongationPluton < 20) {commentPluton = "Inobservable"; quandPluton = " "}
        //
        // if (elongationPluton > 20 && elongationPluton < 45){
        // commentPluton = "Observable aux instruments"
        // if (elongation_planetePluton == "Ouest") {quandPluton = "en toute fin de nuit"}
        // if (elongation_planetePluton == "Est") {quandPluton = "en tout début de soirée"}}
        //
        // if (elongationPluton > 45 && elongationPluton < 120){
        // commentPluton = "Observable aux instruments"
        // if (elongation_planetePluton == "Ouest") {quandPluton = "en seconde partie de nuit"}
        // if (elongation_planetePluton == "Est") {quandPluton = "en première partie de nuit"}}
        //
        // if (elongationPluton > 120 && elongationPluton < 140) {commentPluton = "Observable aux instruments"; quandPluton = "pratiquement toute la nuit"}
        // if (elongationPluton > 140 && elongationPluton < 180) {commentPluton = "Observable aux instruments"; quandPluton = "toute la nuit"}
        //
        //
        // document.calc.visibpluton.value = commentPluton + " " + quandPluton
        //
        // //--------convertion longitude et latitude en ascension droite et déclinaison
        //
        // l = l -y * floor(l /y);
        // l1 = l * r2d;
        // d1 =((l1 - floor(l1)) * 60);
        // d2 =((d1 - floor(d1)) * 60);
        //
        // document.calc.longpluton.value = floor(l1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        //
        // beta = asin(r * sin(b) / distance_terre);
        // b1 = abs(beta * r2d);
        // d1 = ((b1 - floor(b1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        // if (beta < 0) {signe = "-";}
        // else {signe = "+";}
        //
        // document.calc.latpluton.value = signe + floor(b1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // document.calc.rayonpluton.value = floor(r * 100000 + 0.5) / 100000 + " UA"
        // document.calc.distancepluton.value = floor(distance_terre * 100000 + 0.5) / 100000 + " UA"
        //
        // asc = atan((cos(obliquite) * sin(l) - tan(beta) * sin(obliquite)) / cos(l));
        // if (asc < 0) {asc = asc + 2 * PI;}
        // if (cos(l) < 0) {asc = asc + PI;}
        // if (asc > 2 * PI) {asc = asc - 2 * PI;}
        // declin = r2d * asin(sin(beta) * cos(obliquite) + cos(beta) * sin(obliquite) * sin(l));
        // asc = asc * r2d / 15;
        // d1 = ((asc - floor(asc)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.alphapluton.value = floor(asc) + "h" + zero(floor(d1)) + "m" + zero(floor(d2 * 100 + 0.5) / 100) + "s";
        // ADPluton = asc * 15 / r2d
        //
        // d = abs(declin);
        // d1 = ((d - floor(d)) * 60);
        // d2= ((d1 - floor(d1)) * 60);
        //
        // if (declin < 0) {signe = "-"; DecPluton = - d / r2d}
        // else {signe = "+"; DecPluton = + d / r2d}
        //
        // document.calc.deltapluton.value = signe + floor(d) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        // ascPluton = asc
        // declinPluton = declin
        //
        // //Magnitude de la planète
        //
        // dist = R // rayon vecteur Soleil-Terre
        // ray = r // rayon vecteur Soleil-planète
        // delta = distance_terre // distance Terre-planète
        //
        // FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) )
        // FV = (FV) * r2d
        //
        // phase = (1 + cos(FV / r2d)) * 50
        //
        // lgrd = log(ray * delta) / log(10);
        // magnitude = - 0.14 + 5 * lgrd;
        //
        // document.calc.magpluton.value = floor(magnitude * 100) / 100;
        // document.calc.phasepluton.value = floor(phase * 10) / 10 + "%";
        //
        // //--------------------------------------------- Lune
        //
        // ct = 0.00000484814;
        // longitude_soleil = longitude + 0.00134 / r2d * cos(a) + 0.00154 / r2d * cos(b) + 0.002 / r2d * cos(c) + 0.00179 / r2d * sin(d) + 0.00178 / r2d * sin(e);
        // longitude_soleil = longitude_soleil - y * floor(longitude_soleil / y);
        //
        // e = 1 - 0.002516 * T_2000 - 0.0000074 * T2_2000
        // excentricite = e * 0.01675104;
        //
        // mercure = (252.250906 + 149472.674636 * T_2000) / r2d;
        // mercure = mercure - y * floor(mercure /y);
        // venus = (181.979801 + 58517.815676 * T_2000) / r2d;
        // venus = venus - y * floor(venus / y);
        // longitude_moyenne_terre = (100.46645 + 35999.372854 * T_2000) / r2d;
        // longitude_moyenne_terre = longitude_moyenne_terre - y * floor(longitude_moyenne_terre /y);
        // mars = (355.433275 + 19140.299331 * T_2000) / r2d;
        // mars = mars - y * floor(mars / y);
        // jupiter = (34.351484 + 3034.905675 * T_2000) / r2d;
        // jupiter = jupiter - y * floor(jupiter / y);
        // saturne = (50.077471 + 1222.113794 * T_2000) / r2d;
        // saturne = saturne - y * floor(saturne /y);
        // uranus = ((314 + 3 / 60 + 18.01841 / 3600) + 1542481.19393 / 3600 * T_2000) / r2d;
        // uranus = uranus - y * floor(uranus / y);
        // neptune = ((304 + 20 / 60 + 55.19575 / 3600) + 786550.32074 / 3600 * T_2000) / r2d;
        // neptune = neptune - y * floor(neptune / y);
        //
        // a1=(119.75 + 131.849 * T_2000) / r2d;
        // a2=(53.09 + 479264.29 * T_2000) / r2d;
        // a3=(313.45 + 481266.484 * T_2000) / r2d;
        //
        // om = (125.044555 - 1934.1361849 * T_2000 + 0.0020762 * T2_2000 + T3_2000 / 467410 - T4_2000 / 60616000) / r2d;
        // longitude_noeud = om;
        //
        // l = (218.3164591 + 481267.88134236 * T_2000 - 0.0013268 * T2_2000 + T3_2000 / 538841 - T4_2000 / 65194000) / r2d;
        // l = l - y * floor(l / y);
        // w1 = l;
        //
        // m = (357.5291092 + 35999.0502909 * T_2000 - 0.0001536 * T2_2000 + T3_2000 / 24490000) / r2d;
        // l_prime = m;
        // anomalie_lune = m;
        //
        // f = (93.2720993 + 483202.0175273 * T_2000 - 0.0034029 * T2_2000 - T3_2000/3526000 + T4_2000 / 863310000) / r2d;
        // f = f - y * floor(f / y);
        //
        // n = (134.9634114 + 477198.8676313 * T_2000 + 0.008997 * T2_2000 + T3_2000 / 69699 - T4_2000 / 14712000) /r2d;
        // n = n - y * floor(n / y);
        // petit_l = n;
        //
        // d = (297.8502042 + 445267.1115168 * T_2000 - 0.00163 * T2_2000 + T3_2000 / 545868 - T4_2000 / 113065000) / r2d;
        // d = d - y * floor(d / y);
        //
        // dzeta_lune = w1 + 5029.0966 / 3600 / r2d * T_2000;
        //
        // nu = (-171996 - 174.2 * T_2000) * sin(om);
        // nu = nu + (-13187 - 1.6 * T_2000) * sin(-2 * d + 2 * f + 2 * om);
        // nu = nu + (-2274 - 0.2 * T_2000) * sin(2 * f + 2 * om);
        // nu = nu + (2062 + 0.2 * T_2000) * sin(2 * om);
        // nu = nu + (1426 - 3.4 * T_2000) * sin(m);
        // nu = nu + (712 + 0.1 * T_2000) * sin(n);
        // nu = nu + (-517 + 1.2 * T_2000) * sin(-2 * d + m + 2 * f + 2 * om);
        // nu = nu + (-386 - 0.4 * T_2000) * sin(2 * f + om);
        // nu = nu - 301 * sin(n + 2 * f + 2 * om);
        // nu = nu + (217 - 0.5 * T_2000) * sin(-2 * d - m + 2 * f + 2 * om);
        // nu = nu - 158 * sin(-2 * d + n);
        // nu = nu + (129 + 0.1 * T_2000) * sin (-2 * d + 2 * f + om);
        // nu = nu + 123 * sin (-n + 2 * f + 2 * om);
        // nu = nu + 63 * sin(2 * d);
        // nu = nu + (63 + 0.1 * T_2000) * sin (n + om);
        // nu = nu - 59 * sin(2 * d - n + 2 * f + 2 * om);
        // nu = nu - (58 - 0.1 * T_2000) * sin(-n + om);
        // nu = nu - 51 * sin(n + 2 * f + om);
        // nu = nu + 48 * sin(-2 * d + 2 * n);
        // nu = nu + 46 * sin(-2 * n + 2 * f + om);
        // nu = nu - 38 * sin(2 * (d + f + om));
        // nu = nu - 31 * sin(2 * (n + f + om));
        // nu = nu + 29 * sin(2 * n);
        // nu = nu + 29 * sin(-2 * d + n + 2 * f + 2 * om);
        // nu = nu + 26 * sin(2 * f);
        // nu = nu - 22 * sin(-2 * d + 2 * f);
        // nu = nu + 21 * sin(-n + 2 * f + om);
        // nu = nu + (17 - 0.1 * T_2000) * sin(2 * m);
        // nu = nu + 16 * sin(2 * d - n + om);
        // nu = nu - (16 + 0.1 * T_2000) * sin(-2 * d + 2 * m + 2 * f + 2 * om);
        // nu = nu - 15 * sin(m + om);
        // nu = nu - 13 * sin(-2 * d + n + om);
        // nu = nu - 12 * sin(-m - om);
        // nu = nu + 11 * sin(2 * n - 2 * f);
        // nu = nu - 10 * sin(2 * d - n + 2 * f + om);
        // nu = nu - 8 * sin(2 * d + n + 2 * f + 2 * om);
        // nu = nu + 7 * sin(m + 2 * f + 2 * om);
        // nu = nu - 7 * sin(-2 * d + m + n);
        // nu = nu - 7 * sin(-m + 2 * f + 2 * om);
        // nu = nu - 7 * sin(2 * d + 2 * f + om);
        // nu = nu + 6 * sin(2 * d + n);
        // nu = nu + 6 * sin(-2 * d + 2 * n + 2 * f + 2 * om);
        // nu = nu + 6 * sin(-2 * d + n + 2 * f + om);
        // nu = nu - 6 * sin(2 * d - 2 * n + om);
        // nu = nu - 6 * sin(2 * d + om);
        // nu = nu + 5 * sin(-m + n);
        // nu = nu - 5 * sin(-2 * d - m + 2 * f + om);
        // nu = nu - 5 * sin(-2 * d + om);
        // nu = nu - 5 * sin(2 * n + 2 * f + om);
        // nu = nu + 4 * sin(-2 * d + 2 * n + om);
        // nu = nu + 4 * sin(-2 * d + m + 2 * f + om);
        // nu = nu + 4 * sin(n - 2 * f);
        // nu = nu - 4 * sin(-d + n);
        // nu = nu - 4 * sin(-2 * d + m);
        // nu = nu - 4 * sin(d);
        // nu = nu + 3 * sin(n + 2 * f);
        // nu = nu - 3 * sin(-2 * n + 2 * f + 2 * om);
        // nu = nu - 3 * sin(-d - m - n);
        // nu = nu - 3 * sin(m + n);
        // nu = nu - 3 * sin(-m + n + 2 * f + 2 * om);
        // nu = nu - 3 * sin(2 * d - m - n + 2 * f + 2 * om);
        // nu = nu - 3 * sin(3 * n + 2 * f + 2 * om);
        // nu = nu - 3 * sin(2 * d - m + 2 * f + 2 * om);
        //
        // nutation_en_longitude = nu / 10000;
        //
        // nu = (92025 + 8.9 * T_2000) * cos(om);
        // nu = nu + (5736 - 3.1 * T_2000) * cos(-2 * d + 2 * f + 2 * om);
        // nu = nu + (977 - 0.5 * T_2000) * cos(2 * f + 2 * om);
        // nu = nu + (-895 + 0.5 * T_2000) * cos(2 * om);
        // nu = nu + (54 - 0.1 * T_2000) * cos(m);
        // nu = nu - 7 * cos(n);
        // nu = nu + (224 - 0.6 * T_2000) * cos(-2 * d + m + 2 * f + 2 * om);
        // nu = nu + 200 * cos(2 * f + om);
        // nu = nu + (129 - 0.1 * T_2000) * cos(n + 2 * f + 2 * om);
        // nu = nu + (-95 + 0.3 * T_2000) * cos(-2 * d - m + 2 * f + 2 * om);
        // nu = nu - 70 * cos(-2 * d + 2 * f + om);
        // nu = nu - 53 * cos(-n + 2 * f + 2 * om);
        // nu = nu - 33 * cos(n + om);
        // nu = nu + 26 * cos(2 * d - n + 2 * f + 2 * om);
        // nu = nu + 32 * cos(-n + om);
        // nu = nu + 27 * cos(n + 2 * f + om);
        // nu = nu - 24 * cos(-2 * n + 2 * f + om);
        // nu = nu + 16 * cos(2 * (d + f + om));
        // nu = nu + 13 * cos(2 * (n + f + om));
        // nu = nu - 12 * cos(-2 * d + n + 2 * f + 2 * om);
        // nu = nu - 10 * cos(-n + 2 * f + om);
        // nu = nu - 8 * cos(2 * d - n + om);
        // nu = nu + 7 * cos(-2 * d + 2 * m + 2 * f + 2 * om);
        // nu = nu + 9 * cos(m + om);
        // nu = nu + 7 * cos(-2 * d + n + om);
        // nu = nu + 6 * cos(-m + om);
        // nu = nu + 5 * cos(2 * d - n + 2 * f + om);
        // nu = nu + 3 * cos(2 * d + n + 2 * f + 2 * om);
        // nu = nu - 3 * cos(m + 2 * f + 2 * om);
        // nu = nu + 3 * cos(-m + 2 * f + 2 * om);
        // nu = nu + 3 * cos(2 * d + 2 * f + om);
        // nu = nu - 3 * cos(-2 * d + 2 * n + 2 * f + 2 * om);
        // nu = nu - 3 * cos(-2 * d + n + 2 * f + om);
        // nu = nu + 3 * cos(2 * d - 2 * n + om);
        // nu = nu + 3 * cos(2 * d + om);
        // nu = nu + 3 * cos(-2 * d - m + 2 * f + om);
        // nu = nu + 3 * cos(-2 * d + om);
        // nu = nu + 3 * cos(2 * n + 2 * f + om);
        //
        // nutation_en_obliquite = nu / 10000;
        //
        // obliquite = (23 + 26 / 60 + 21.448 / 3600 - 46.815 / 3600 * T_2000 - 0.00059 / 3600 * T2_2000 + 0.001813 / 3600 * T3_2000) / r2d + nutation_en_obliquite * ct;
        //
        // correct = 22639.55 * sin (n);
        // correct = correct + 4586.43061 * sin (2 * d - n);
        // correct = correct + 2369.91227 * sin (2 * d);
        // correct = correct + 769.02326 * sin (2 * n);
        // correct = correct + 211.65487 * sin (2 * d - 2 * n);
        // correct = correct + 205.44315 * sin (2 * d - m - n);
        // correct = correct + 191.95575 * sin (2 * d + n);
        // correct = correct + 164.73458 * sin (2 * d - m);
        // correct = correct + 55.17801 * sin (2 * d - 2 * f);
        // correct = correct + 39.53393 * sin (n - 2 * f);
        // correct = correct + 38.42974 * sin (4 * d - n);
        // correct = correct + 36.12364 * sin (3 * n);
        // correct = correct + 30.77247 * sin (4 * d - 2 * n);
        // correct = correct + 17.95512 * sin (d + m);
        // correct = correct + 14.53078 * sin (2 * d - m + n);
        // correct = correct + 14.37964 * sin (2 * d + 2 * n);
        // correct = correct + 13.89903 * sin (4 * d);
        // correct = correct + 13.194 * sin (2 * d - 3 * n);
        // correct = correct + 8.60582 * sin (2 * d - m - 2 * n);
        // correct = correct + 8.05076 * sin (2 * d - 2 * m);
        // correct = correct + 7.37173 * sin (2 * d - 2 * m - n);
        // correct = correct + 4.37416 * sin (4 * d - m - n);
        // correct = correct + 2.73198 * sin (4 * d - m - 2 * n);
        // correct = correct + 2.48897 * sin (2 * d + m - 2 * n);
        // correct = correct + 2.14619 * sin (2 * d - m - 2 * f);
        // correct = correct + 1.97772 * sin (4 * d + n);
        // correct = correct + 1.93367 * sin (4 * n);
        // correct = correct + 1.87083 * sin (4 * d - m);
        // correct = correct + 1.26186 * sin (d + m + n);
        // correct = correct + 1.18682 * sin (4 * d - 3 * n);
        // correct = correct + 1.17704 * sin (2 * d - m + 2 * n);
        // correct = correct + 1.07773 * sin (d + m - n);
        // correct = correct + 1.05949 * sin (2 * d + 3 * n);
        // correct = correct + .94827 * sin (2 * d - 4 * n);
        // correct = correct + .75173 * sin (2 * d - 2 * m + n);
        // correct = correct + .57156 * sin (6 * d - 2 * n);
        // correct = correct + .47842 * sin (2 * d - m - 3 * n);
        // correct = correct + .42034 * sin (4 * f);
        // correct = correct + .41342 * sin (m + 2 * f);
        // correct = correct + .40423 * sin (3 * d);
        // correct = correct + .39451 * sin (6 * d - n);
        // correct = correct + .34966 * sin (d + m - 2 * n);
        // correct = correct + .33983 * sin (2 * d - 3 * m);
        // correct = correct + .30874 * sin (4 * d - 2 * m - n);
        // correct = correct + .30157 * sin (m - n - 2 * f);
        // correct = correct + .30086 * sin (4 * d - n - 2 * f);
        // correct = correct + .29422 * sin (2 * d - 2 * m - 2 * n);
        // correct = correct + .29255 * sin (6 * d - 3 * n);
        // correct = correct + .28251 * sin (4 * d - m + n);
        // correct = correct + .27377 * sin (3 * d + m - n);
        // correct = correct + .26338 * sin (m + n + 2 * f);
        // correct = correct + .25429 * sin (d + 2 * f);
        // correct = correct + .24697 * sin (2 * d - 3 * m - n);
        // correct = correct + .21853 * sin (4 * d + 2 * n);
        // correct = correct + .17903 * sin (2 * d - n - 2 * f);
        // correct = correct + .17624 * sin (2 * d + m - 3 * n);
        // correct = correct + .15781 * sin (4 * d - 2 * m - 2 * n);
        // correct = correct + .15227 * sin (4 * d - 2 * m);
        // correct = correct + .1499 * sin (3 * d + m);
        // correct = correct + .12616 * sin (6 * d);
        // correct = correct + .111 * sin (5 * n);
        // correct = correct + .09982 * sin (4 * d - m - 3 * n);
        // correct = correct + .0932 * sin (2 * d - m + 3 * n);
        // correct = correct + .09205 * sin (d + m + 2 * n);
        // correct = correct + .09092 * sin (n + 4 * f);
        // correct = correct + .09033 * sin (6 * d - m - 2 * n);
        // correct = correct + .08472 * sin (2 * d + m + n - 2 * f);
        // correct = correct + .07765 * sin (2 * d + 4 * n);
        // correct = correct + .07501 * sin (m - 2 * f);
        // correct = correct + .07142 * sin (6 * d - m - n);
        // correct = correct + .0685 * sin (2 * d - 5 * n);
        // correct = correct + .06742 * sin (2 * d + m - n + 2 * f);
        // correct = correct + .06541 * sin (2 * d + m + 2 * f);
        // correct = correct + .06507 * sin (3 * d - m);
        // correct = correct + .06439 * sin (2 * d - 2 * m + 2 * n);
        // correct = correct + .06314 * sin (2 * d - 2 * m - 2 * f);
        // correct = correct + .05165 * sin (m - 2 * n - 2 * f);
        // correct = correct + .0445 * sin (d + n + 2 * f);
        // correct = correct + .04338 * sin (m + 2 * n + 2 * f);
        // correct = correct + .04304 * sin (d - 2 * m);
        // correct = correct + .039 * sin (6* d - m - 3 * n);
        // correct = correct + .033 * sin (2 * d - 3 * m + n);
        // correct = correct + .03274 * sin (4 * d - m + 2 * n);
        // correct = correct + .02949 * sin (2 * d - m - 4 * n);
        // correct = correct + .02682 * sin (4 * d + m - 3 * n);
        // correct = correct + .02677 * sin (m + 2 * n - 2 * f);
        // correct = correct + .0251 * sin (6 * d - m);
        // correct = correct + .02429 * sin (m - 2 * n + 2 * f);
        // correct = correct + .02411 * sin (4 * d - 2 * m + n);
        // correct = correct + .02296 * sin (d + m - 3 * n);
        // correct = correct + .02289 * sin (4 * d - m - n - 2 * f);
        // correct = correct + .02285 * sin (6 * d + n);
        // correct = correct + .02244 * sin (3 * d + m + n);
        // correct = correct + .02149 * sin (4 * d + 3 * n);
        // correct = correct + .01993 * sin (2 * d - n + 4 * f);
        // correct = correct + .01819 * sin (2 * d + m - 4 * n);
        // correct = correct + .01741 * sin (4 * d - 3 * m - n);
        // correct = correct + .01605 * sin (2 * d + m + n + 2 * f);
        // correct = correct + .01598 * sin (d - n + 2 * f);
        // correct = correct + .01544 * sin (2 * d - 2 * m - 3 * n);
        // correct = correct + .01376 * sin (6 * d - 4 * n);
        // correct = correct + .01372 * sin (2 * d + 4 * f);
        // correct = correct + .01331 * sin (2 * d - 4 * m);
        // correct = correct + .01297 * sin (2 * n + 4 * f);
        // correct = correct + .01215 * sin (3 * d - n + 2 * f);
        // correct = correct + .00971 * sin (4 * d - 3 * m);
        // correct = correct + .00965 * sin (2 * d - 3 * m - 2 * n);
        // correct = correct + .00891 * sin (3 * d + m - 2 * f);
        // correct = correct + .00889 * sin (2 * d + m + 2 * n - 2 * f);
        // correct = correct + .00866 * sin (8 * d - 2 * n);
        // correct = correct + .0084 * sin (8 * d - 3 * n);
        // correct = correct + .00836 * sin (6 * d - 2 * m - 2 * n);
        // correct = correct + .00812 * sin (2 * d - 4 * m - n);
        // correct = correct + .00755 * sin (4 * d - 3 * m - 2 * n);
        // correct = correct + .00744 * sin (6 * d - 2 * m - n);
        // correct = correct + .0073 * sin (2 * d - m + 4 * n);
        // correct = correct + .00679 * sin (d + m + 3 * n);
        // correct = correct + .00666 * sin (4 * d - m - 2 * f);
        // correct = correct + .00665 * sin (6 * n);
        // correct = correct + .00662 * sin (4 * d - 2 * n - 2 * f);
        // correct = correct + .00623 * sin (m - 3 * n - 2 * f);
        // correct = correct + .00568 * sin (2 * d + 5 * n);
        // correct = correct + .0056 * sin (4 * d - 2 * m - 3 * n);
        // correct = correct + .0054 * sin (d + 2 * n + 2 * f);
        // correct = correct + .00538 * sin (2 * d - 2 * m + 3 * n);
        // correct = correct + .00526 * sin (m + 3 * n + 2 * f);
        // correct = correct + .00519 * sin (2 * m + 2 * f);
        // correct = correct + .00518 * sin (3 * d - 2 * m);
        // correct = correct + .00515 * sin (2 * d + 2 * m - n + 2 * f);
        // correct = correct + .00497 * sin (2 * d - 6 * n);
        // correct = correct + .00477 * sin (6 * d - m + n);
        // correct = correct + .00475 * sin (5 * d + m - n);
        // correct = correct + .00473 * sin (2 * m - n - 2 * f);
        // correct = correct + .00467 * sin (2 * d - 3 * n + 2 * f);
        // correct = correct + .00455 * sin (8 * d - n);
        // correct = correct + .00439 * sin (5 * d);
        // correct = correct + .00392 * sin (5 * d + m - 2 * n);
        // correct = correct + .00375 * sin (3 * d + 2 * f);
        // correct = correct + .00364 * sin (6 * d - 2 * n - 2 * f);
        // correct = correct + .00361 * sin (d + 2 * m - 2 * n);
        // correct = correct + .00353 * sin (4 * d + m - n + 2 * f);
        // correct = correct + .00344 * sin (2 * d + n + 4 * f);
        // correct = correct + .00336 * sin (4 * d - m + 3 * n);
        // correct = correct + .0033 * sin (3 * d - m + n);
        // correct = correct + .00324 * sin (8 * d - 4 * n);
        // correct = correct + .00318 * sin (6 * d + 2 * n);
        // correct = correct + .00312 * sin (6 * d - 2 * m - 3 * n);
        // correct = correct + .00298 * sin (3 * d - 2 * n + 2 * f);
        // correct = correct + .00295 * sin (2 * d - 3 * m + 2 * n);
        // correct = correct + .0029 * sin (4 * d - 2 * m + 2 * n);
        // correct = correct + .00289 * sin (d - 2 * n - 2 * f);
        // correct = correct + .00285 * sin (6 * d - 2 * m);
        // correct = correct + .00282 * sin (2 * d - 2 * n + 4 * f);
        // correct = correct + .0027 * sin (2 * m + n + 2 * f);
        // correct = correct + .00262 * sin (2 * d + m + 2 * n + 2 * f);
        // correct = correct + .00256 * sin (3 * d + m + 2 * n);
        // correct = correct + .00254 * sin (d - 3 * m);
        // correct = correct + .00229 * sin (d - 2 * m - n);
        // correct = correct + .0022 * sin (4 * d + m - 2 * n + 2 * f);
        // correct = correct + .00198 * sin (2 * d + m - 4 * f);
        // correct = correct + .00198 * sin (4 * d + 4 * n);
        // correct = correct + .00196 * sin (8 * d - m - 2 * n);
        // correct = correct + .00186 * sin (4 * d + m + 2 * f);
        // correct = correct + .00183 * sin (4 * d + m + n - 2 * f);
        // correct = correct + .00181 * sin (5 * d + m);
        // correct = correct + .00178 * sin (2 * d - m - 5 * n);
        // correct = correct + .00176 * sin (6 * d - m - 4 * n);
        // correct = correct + .00173 * sin (2 * d + m - 5 * n);
        // correct = correct + .0017 * sin (8 * d - m - 3 * n);
        // correct = correct + .00166 * sin (m + 3 * n - 2 * f);
        // correct = correct + .00163 * sin (2 * d - 3 * m - 2 * f);
        // correct = correct + .0016 * sin (4 * d - 3 * m + n);
        // correct = correct + .00155 * sin (d - m + 2 * f);
        // correct = correct + .00155 * sin (d + m - 4 * n);
        // correct = correct + .00153 * sin (3 * n + 4 * f);
        // correct = correct + .00139 * sin (8 * d);
        // correct = correct + .00133 * sin (2 * d - 4 * m + n);
        // correct = correct + .00123 * sin (d - 4 * f);
        // correct = correct + .00116 * sin (3 * d + m - n - 2 * f);
        // correct = correct + .00112 * sin (8 * d - m - n);
        // correct = correct + .00108 * sin (4 * d - 2 * m - n - 2 * f);
        // correct = correct + .00106 * sin (m - 3 * n + 2 * f);
        // correct = correct + .00102 * sin (5 * d - m);
        // correct = correct + .001 * sin (2 * m - 2 * n - 2 * f);
        // correct = correct + .00096 * sin (2 * d + 2 * m + 2 * f);
        //
        // correct = correct - 666.44186 * sin (m);
        // correct = correct - 411.60287 * sin (2 * f);
        // correct = correct - 147.32654 * sin (m - n);
        // correct = correct - 124.98806 * sin (d);
        // correct = correct - 109.38419 * sin (m + n);
        // correct = correct - 45.10032 * sin (n + 2 * f);
        // correct = correct - 28.3981 * sin (2 * d + m - n);
        // correct = correct - 24.3591 * sin (2 * d + m);
        // correct = correct - 18.58467 * sin (d - n);
        // correct = correct - 9.67938 * sin (m - 2 * n);
        // correct = correct - 9.36601 * sin (2 * d - n + 2 * f);
        // correct = correct - 8.45308 * sin (d + n);
        // correct = correct - 7.63041 * sin (m + 2 * n);
        // correct = correct - 7.44804 * sin (2 * m);
        // correct = correct - 6.38325 * sin (2 * d + n - 2 * f);
        // correct = correct - 5.7417 * sin (2 * d + 2 * f);
        // correct = correct - 3.99767 * sin (2 * n + 2 * f);
        // correct = correct - 3.20968 * sin (3 * d - n);
        // correct = correct - 2.91464 * sin (2 * d + m + n);
        // correct = correct - 2.56813 * sin (2 * m - n);
        // correct = correct - 2.52138 * sin (2 * d + 2 * m - n);
        // correct = correct - 1.75296 * sin (d - 2 * n);
        // correct = correct - 1.43724 * sin (2 * d + m - 2 * f);
        // correct = correct - 1.37259 * sin (2 * n - 2 * f);
        // correct = correct - 1.22412 * sin (3 * d - 2 * n);
        // correct = correct - 1.16177 * sin (2 * m + n);
        // correct = correct - .99023 * sin (2 * d + n + 2 * f);
        // correct = correct - .6694 * sin (m - 3 * n);
        // correct = correct - .63523 * sin (4 * d + m - n);
        // correct = correct - .58399 * sin (d + 2 * n);
        // correct = correct - .58332 * sin (d - 2 * f);
        // correct = correct - .56065 * sin (2 * d - 2 * n - 2 * f);
        // correct = correct - .55694 * sin (d - m);
        // correct = correct - .54594 * sin (m + 3 * n);
        // correct = correct - .53572 * sin (2* d - 2 * n + 2 * f);
        // correct = correct - .4538 * sin (2 * d + 2 * n - 2 * f);
        // correct = correct - .42624 * sin (2 * d - m - n + 2 * f);
        // correct = correct - .38215 * sin (2 * d - m + 2 * f);
        // correct = correct - .37453 * sin (2 * d - m + n - 2 * f);
        // correct = correct - .35759 * sin (4 * d + m - 2 * n);
        // correct = correct - .32866 * sin (3 * n + 2 * f);
        // correct = correct - .29023 * sin (2 * d + m + 2 * n);
        // correct = correct - .28911 * sin (4 * d + m);
        // correct = correct - .25304 * sin (3 * d - 2 * f);
        // correct = correct - .2499 * sin (2 * d + 2 * m - 2 * n);
        // correct = correct - .23141 * sin (3 * d - m - n);
        // correct = correct - .20134 * sin (4 * d - n + 2 * f);
        // correct = correct - .19311 * sin (2 * m - 2 * n);
        // correct = correct - .18576 * sin (2 * d + 2 * m);
        // correct = correct - .16977 * sin (4 * d - 2 * n + 2 * f);
        // correct = correct - .13636 * sin (d - m - n);
        // correct = correct - .12812 * sin (d - 3 * n);
        // correct = correct - .12386 * sin (2 * d + 2 * n + 2 * f);
        // correct = correct - .12073 * sin (d - m + n);
        // correct = correct - .10136 * sin (3 * m);
        // correct = correct - .09154 * sin (2 * d - 3 * n - 2 * f);
        // correct = correct - .085 * sin (4 * d + 2 * f);
        // correct = correct - .08311 * sin (3 * d - m - 2 * n);
        // correct = correct - .08282 * sin (m + n - 2 * f);
        // correct = correct - .08049 * sin (m - n + 2 * f);
        // correct = correct - .08019 * sin (n - 4 * f);
        // correct = correct - .07518 * sin (2 * d - 4 * f);
        // correct = correct - .07373 * sin (2 * d - m + n + 2 * f);
        // correct = correct - .06601 * sin (4 * d + n - 2 * f);
        // correct = correct - .06513 * sin (2 * m + 2 * n);
        // correct = correct - .06103 * sin (2 * d - m - n - 2 * f);
        // correct = correct - .05725 * sin (5 * d - 2 * n);
        // correct = correct - .05684 * sin (3 * n - 2 * f);
        // correct = correct - .05142 * sin (3 * m - n);
        // correct = correct - .0507 * sin (4 * d + m + n);
        // correct = correct - .04702 * sin (m - 4 * n);
        // correct = correct - .04442 * sin (3 * d - 3 * n);
        // correct = correct - .04189 * sin (3 * d + m - 2 * n);
        // correct = correct - .04074 * sin (d + 3 * n);
        // correct = correct - .04012 * sin (d + n - 2 * f);
        // correct = correct - .03968 * sin (d + 2 * m);
        // correct = correct - .03947 * sin (m + 4 * n);
        // correct = correct - .03587 * sin (d + m + 2 * f);
        // correct = correct - .03514 * sin (4 * d + 2 * m - 2 * n);
        // correct = correct - .03336 * sin (2 * d + 3 * n - 2 * f);
        // correct = correct - .02979 * sin (3 * d - n - 2 * f);
        // correct = correct - .02887 * sin (2 * d - m + 2 * n - 2 * f);
        // correct = correct - .02804 * sin (2 * d - m - 2 * n - 2 * f);
        // correct = correct - .02676 * sin (2 * d + m + 3 * n);
        // correct = correct - .02602 * sin (4 * n + 2 * f);
        // correct = correct - .02391 * sin (4 * d - 2 * f);
        // correct = correct - .02379 * sin (d - n - 2 * f);
        // correct = correct - .02349 * sin (2 * d + 2 * m - 2 * f);
        // correct = correct - .02273 * sin (4 * d - m - n + 2 * f);
        // correct = correct - .02171 * sin (4 * d + 2 * m - n);
        // correct = correct - .02157 * sin (2 * d - m - 2 * n + 2 * f);
        // correct = correct - .01948 * sin (3 * d - m - 2 * f);
        // correct = correct - .01875 * sin (4 * d + m - n - 2 * f);
        // correct = correct - .01816 * sin (2 * d - 2 * m + 2 * f);
        // correct = correct - .01796 * sin (3 * m + n);
        // correct = correct - .01781 * sin (4 * d + n + 2 * f);
        // correct = correct - .01686 * sin (5 * d - 3 * n);
        // correct = correct - .01644 * sin (2 * d - 2 * m + n - 2 * f);
        // correct = correct - .01541 * sin (2 * d - 2 * m - n + 2 * f);
        // correct = correct - .01533 * sin (4 * d - m - 2 * n + 2 * f);
        // correct = correct - .01514 * sin (2 * m - 3 * n);
        // correct = correct - .01483 * sin (d - m + 2 * n);
        // correct = correct - .0135 * sin (5 * d - n);
        // correct = correct - .01343 * sin (2 * d + 2 * m + n);
        // correct = correct - .01332 * sin (2 * d + 3 * n + 2 * f);
        // correct = correct - .01282 * sin (6 * d + m - 2 * n);
        // correct = correct - .01281 * sin (d - m - 2 * f);
        // correct = correct - .01182 * sin (3 * d - 2 * m - n);
        // correct = correct - .01114 * sin (4 * d - m + 2 * f);
        // correct = correct - .01077 * sin (2 * d - 4 * n - 2 * f);
        // correct = correct - .01064 * sin (6 * d + m - n);
        // correct = correct - .01062 * sin (3 * d + n - 2 * f);
        // correct = correct - .01007 * sin (2 * d - m + 2 * n + 2 * f);
        // correct = correct - .0098 * sin (4 * d + 2 * n - 2 * f);
        // correct = correct - .00955 * sin (d - 4 * n);
        // correct = correct - .00944 * sin (2 * d + 2 * m - 3 * n);
        // correct = correct - .00934 * sin (4 * d - 3 * n + 2 * f);
        // correct = correct - .0085 * sin (2 * d - n - 4 * f);
        // correct = correct - .00849 * sin (d + 2 * m + n);
        // correct = correct - .00732 * sin (4 * d - m + n - 2 * f);
        // correct = correct - .00694 * sin (d - m - 2 * n);
        // correct = correct - .00693 * sin (5 * d - m - 2 * n);
        // correct = correct - .00668 * sin (4 * d + m + 2 * n);
        // correct = correct - .00659 * sin (d + m + n + 2 * f);
        // correct = correct - .00654 * sin (2 * d + 2 * m - n - 2 * f);
        // correct = correct - .00623 * sin (3 * d + m - 3 * n);
        // correct = correct - .00509 * sin (6 * d - 2 * n + 2 * f);
        // correct = correct - .00478 * sin (6 * d + m - 3 * n);
        // correct = correct - .00434 * sin (2 * d - 2 * m - n - 2 * f);
        // correct = correct - .00431 * sin (4 * d - 5 * n);
        // correct = correct - .00416 * sin (3 * m - 2 * n);
        // correct = correct - .00399 * sin (3 * d - 2 * m - 2 * n);
        // correct = correct - .00396 * sin (6 * d + m);
        // correct = correct - .00389 * sin (3 * d + 2 * n);
        // correct = correct - .00378 * sin (2 * d - 2 * m + n + 2 * f);
        // correct = correct - .00369 * sin (4 * d + 2 * m - 3 * n);
        // correct = correct - .00365 * sin (2 * d - m - 3 * n - 2 * f);
        // correct = correct - .00359 * sin (6 * d - n + 2 * f);
        // correct = correct - .00355 * sin (2 * m - 2 * f);
        // correct = correct - .00354 * sin (4 * n - 2 * f);
        // correct = correct - .00346 * sin (2 * d + m - 2 * n - 2 * f);
        // correct = correct - .00341 * sin (2 * m + 3 * n);
        // correct = correct - .00335 * sin (5 * d - n - 2 * f);
        // correct = correct - .00332 * sin (m - 5 * n);
        // correct = correct - .003 * sin ( d + 2 * m - n);
        // correct = correct - .00297 * sin (3 * d - m - 3 * n);
        // correct = correct - .00287 * sin (m + 5 * n);
        // correct = correct - .00287 * sin (6 * d - 3 * n + 2 * f);
        // correct = correct - .00286 * sin (2 * d - m - 4 * f);
        // correct = correct - .00285 * sin (d + 4 * n);
        // correct = correct - .00274 * sin (4 * d + 2 * n + 2 * f);
        // correct = correct - .00251 * sin (4 * d - m + n + 2 * f);
        // correct = correct - .00247 * sin (2 * d + 4 * n - 2 * f);
        // correct = correct - .00236 * sin (2 * d + m + 4 * n);
        // correct = correct - .00232 * sin (2 * d - m + 3 * n - 2 * f);
        // correct = correct - .00228 * sin (2 * d + m - n - 2 * f);
        // correct = correct - .00214 * sin (6 * d - 2 * f);
        // correct = correct - .00212 * sin (d - m + n - 2 * f);
        // correct = correct - .00208 * sin (4 * d + 2 * m);
        // correct = correct - .00201 * sin (5 * n + 2 * f);
        // correct = correct - .002 * sin (2 * d + 2 * m + n - 2 * f);
        // correct = correct - .00191 * sin (3 * d + 2 * m);
        // correct = correct - .00189 * sin (3 * d - m - n - 2 * f);
        // correct = correct - .00189 * sin (5 * d - m - 3 * n);
        // correct = correct - .00188 * sin (2 * d + 3 * m - n);
        // correct = correct - .00174 * sin (3 * d - 4 * n);
        // correct = correct - .0016 * sin (4 * d - 2 * m - n + 2 * f);
        // correct = correct - .00157 * sin (d + m + n - 2 * f);
        // correct = correct - .00154 * sin (5 * d - m - n);
        // correct = correct - .00149 * sin (d - m + 3 * n);
        // correct = correct - .00142 * sin (d - 2 * n + 2 * f);
        // correct = correct - .00138 * sin (3 * d + m - n + 2 * f);
        // correct = correct - .00137 * sin (5 * d - 2 * f);
        // correct = correct - .00133 * sin (2 * d - 2 * m + 2 * n - 2 * f);
        // correct = correct - .00132 * sin (6 * d + 2 * f);
        // correct = correct - .00131 * sin (2 * d + 4 * n + 2 * f);
        // correct = correct - .00128 * sin (4 * m);
        // correct = correct - .00127 * sin (3 * d + 2 * m - n);
        // correct = correct - .00121 * sin (4 * d - m + 2 * n - 2 * f);
        // correct = correct - .00119 * sin (2 * m - 4 * n);
        // correct = correct - .00117 * sin (2 * d - m + 3 * n + 2 * f);
        // correct = correct - .00116 * sin (2 * d + m - 3 * n - 2 * f);
        // correct = correct - .00111 * sin (2 * d - 2 * m - 2 * n - 2 * f);
        // correct = correct - .00111 * sin (2 * d - 5 * n - 2 * f);
        // correct = correct - .00109 * sin (4 * d + 3 * n - 2 * f);
        // correct = correct - .00108 * sin (4 * m - n);
        // correct = correct - .00102 * sin (d + 2 * m + 2 * n);
        // correct = correct - .00102 * sin (3 * d - 2 * m - 2 * f);
        // correct = correct - .001 * sin (d - m - n - 2 * f);
        // correct = correct - .00098 * sin (7 * d - 3 * n);
        //
        // correct = correct + 14.2488 * sin (18 * venus - 16 * longitude_moyenne_terre - n + 26.54261 / r2d);
        // correct = correct + 1.1431 * sin (2 * longitude_moyenne_terre - 2 * jupiter + 2 * d - n + 180.11977 / r2d);
        // correct = correct + 0.9011 * sin (4 * longitude_moyenne_terre - 8 * mars + 3 * jupiter + 285.98707 / r2d);
        // correct = correct + 0.8216 * sin (venus - longitude_moyenne_terre + 180.00988 / r2d);
        // correct = correct + 0.7881 * sin (18 * venus - 16 * longitude_moyenne_terre - 2 * n + 26.54324 / r2d);
        // correct = correct + 0.7393 * sin (18 * venus - 16 * longitude_moyenne_terre + 26.54560 / r2d);
        // correct = correct + 0.6437 * sin (3 * venus - 3 * longitude_moyenne_terre + 2 * d - n + 179.98144 / r2d);
        // correct = correct + 0.6388 * sin (longitude_moyenne_terre - jupiter + 1.22890 / r2d);
        // correct = correct + 0.5634 * sin (10 * venus - 3 * longitude_moyenne_terre - n + 333.30551 / r2d);
        // correct = correct + 0.4453 * sin (2 * longitude_moyenne_terre - 3 * jupiter + 2 * d - n + 10.07001 / r2d);
        // correct = correct + 0.3436 * sin (2 * venus - 3 * longitude_moyenne_terre + 269.95393 / r2d);
        // correct = correct + 0.3246 * sin (longitude_moyenne_terre - 2 * mars + 318.13776 / r2d);
        // correct = correct + 0.3016 * sin (2 * venus - 2 * longitude_moyenne_terre + 0.20448 / r2d);
        //
        // correct = correct + 7.06304 * sin (dzeta_lune - f + 0.00094 / r2d);
        // correct = correct + 0.49331 * sin (dzeta_lune + petit_l - f + 0.00127 / r2d);
        // correct = correct + 0.49141 * sin (dzeta_lune - petit_l - f + 0.00127 / r2d);
        // correct = correct + 0.36061 * sin (dzeta_lune + f + 0.00071 / r2d);
        // correct = correct + 0.09642 * sin (dzeta_lune + 2 * d - f + 0.0009 / r2d);
        // correct = correct + 0.06569 * sin (dzeta_lune - 2 * d - f + 0.001 / r2d);
        // correct = correct + 0.06456 * sin (dzeta_lune + 2 * d - petit_l - f + 0.00042 / r2d);
        // correct = correct + 0.05036 * sin (dzeta_lune - petit_l + f + 0.00051 / r2d);
        // correct = correct + 0.04962 * sin (dzeta_lune - 2*d + petit_l - f + 0.00029 / r2d);
        // correct = correct + 0.04746 * sin (dzeta_lune - 2 * d + f + 0.00076 / r2d);
        // correct = correct + 0.03838 * sin (dzeta_lune + petit_l + f + 0.0007 / r2d);
        // correct = correct + 0.03638 * sin (2 * dzeta_lune - 2 * f + 180 / r2d);
        // correct = correct + 0.03402 * sin (dzeta_lune + 2 * petit_l - f + 0.00126 / r2d);
        // correct = correct + 0.03279 * sin (dzeta_lune - 2 * petit_l - f + 0.00128 / r2d);
        // correct = correct + 0.02206 * sin (2 * d - petit_l);
        // correct = correct + 0.01492 * sin (dzeta_lune - 3 * f + 180.00086 / r2d);
        // correct = correct + 0.01234 * sin (dzeta_lune + 2 * d + petit_l - f + 0.00102 / r2d);
        //
        // l = l + correct / 3600 / r2d + nutation_en_longitude * ct;
        // longitude = l;
        // longitude_lune = l;
        //
        // l1 = l * r2d;
        //
        // d1 = ((l1 - floor(l1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.longlune.value = floor(l1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100)+ "''";
        //
        // correct = 18461.4 * sin (f);
        // correct = correct -6.29664 * sin (3 * f);
        // correct = correct + 2.79871 * sin (n - 3 * f);
        // correct = correct + 999.70079 * sin (n - f);
        // correct = correct + 1010.1743 * sin (n + f);
        // correct = correct - 1.01941 * sin (n + 3 * f);
        // correct = correct - .13035 * sin (2 * n - 3 * f);
        // correct = correct + 31.75985 * sin (2 * n - f);
        // correct = correct + 61.91229 * sin (2 * n + f);
        // correct = correct - .11787 * sin (2 * n + 3 * f);
        // correct = correct + 1.58131 * sin (3 * n - f);
        // correct = correct + 3.98407 * sin (3 * n + f);
        // correct = correct - .01181 * sin (3 * n + 3 * f);
        // correct = correct + .09157 * sin (4 * n - f);
        // correct = correct + .26325 * sin (4 * n + f);
        // correct = correct + .01768 * sin (5 * n + f);
        // correct = correct - .07479 * sin (m - 3 * n - f);
        // correct = correct - .02365 * sin (m - 3 * n + f);
        // correct = correct - .79322 * sin (m - 2 * n - f);
        // correct = correct - .30129 * sin (m - 2 * n + f);
        // correct = correct - 6.73173 * sin (m - n - f);
        // correct = correct - 5.6326 * sin (m - n + f);
        // correct = correct - 4.83983 * sin (m - f);
        // correct = correct - 6.46036 * sin (m + f);
        // correct = correct + .01157 * sin (m + 3 * f);
        // correct = correct - 5.07614 * sin (m + n - f);
        // correct = correct - 5.31151 * sin (m + n + f);
        // correct = correct - .31292 * sin (m + 2 * n - f);
        // correct = correct - .63884 * sin (m + 2 * n + f);
        // correct = correct - .02419 * sin (m + 3 * n - f);
        // correct = correct - .06176 * sin (m + 3 * n + f);
        // correct = correct - .01571 * sin (2 * m - 2 * n - f);
        // correct = correct - .11335 * sin (2 * m - n - f);
        // correct = correct - .09511 * sin (2 * m - n + f);
        // correct = correct - .01801 * sin (2 * m - f);
        // correct = correct - .05729 * sin (2 * m + f);
        // correct = correct - .06187 * sin (2 * m + n - f);
        // correct = correct - .05504 * sin (2 * m + n + f);
        // correct = correct + .01031 * sin (d - m - n - f);
        // correct = correct - .01346 * sin (d - m - f);
        // correct = correct - .01829 * sin (d - m + f);
        // correct = correct - .02012 * sin (d - m + n - f);
        // correct = correct - .01255 * sin (d - 3 * n - f);
        // correct = correct - .10964 * sin (d - 2 * n - f);
        // correct = correct - .07846 * sin (d - 2 * n + f);
        // correct = correct - .42989 * sin (d - n - f);
        // correct = correct + .13928 * sin (d - n + f);
        // correct = correct - .03226 * sin (d - 3 * f);
        // correct = correct - 4.80578 * sin (d - f);
        // correct = correct - 5.36844 * sin (d + f);
        // correct = correct - .58893 * sin (d + n - f);
        // correct = correct - .66741 * sin (d + n + f);
        // correct = correct - .03636 * sin (d + 2 * n - f);
        // correct = correct - .06383 * sin (d + 2 * n + f);
        // correct = correct + .01597 * sin (d + m - 2 * n - f);
        // correct = correct + .0168 * sin (d + m - 2 * n + f);
        // correct = correct - .0559 * sin (d + m - n + f);
        // correct = correct + .80426 * sin (d + m - f);
        // correct = correct + .80263 * sin (d + m + f);
        // correct = correct + .03465 * sin (d + m + n - f);
        // correct = correct + .10176 * sin (d + m + n + f);
        // correct = correct + .01016 * sin (d + m + 2 * n + f);
        // correct = correct + .01042 * sin (2 * d - 3 * m - n + f);
        // correct = correct + .03647 * sin (2 * d - 3 * m - f);
        // correct = correct + .01603 * sin (2 * d - 3 * m + f);
        // correct = correct + .02285 * sin (2 * d - 2 * m - 2 * n - f);
        // correct = correct + .26865 * sin (2 * d - 2 * m - n - f);
        // correct = correct + .31474 * sin (2 * d - 2 * m - n + f);
        // correct = correct + 1.08587 * sin (2 * d - 2 * m - f);
        // correct = correct + .38353 * sin (2 * d - 2 * m + f);
        // correct = correct + .06915 * sin (2 * d - 2 * m + n - f);
        // correct = correct + .05848 * sin (2 * d - 2 * m + n + f);
        // correct = correct + .05502 * sin (2 * d - m - 3 * n - f);
        // correct = correct + .65025 * sin (2 * d - m - 2 * n - f);
        // correct = correct - .06208 * sin (2 * d - m - 2 * n + f);
        // correct = correct + .01034 * sin (2 * d - m - n - 3 * f);
        // correct = correct + 7.43488 * sin (2 * d - m - n - f);
        // correct = correct + 8.86853 * sin (2 * d - m - n + f);
        // correct = correct - .01177 * sin (2 * d - m - n + 3 * f);
        // correct = correct + .08815 * sin (2 * d - m - 3 * f);
        // correct = correct + 29.57794 * sin (2 * d - m - f);
        // correct = correct + 7.95891 * sin (2 * d - m + f);
        // correct = correct - .01669 * sin (2 * d - m + n - 3 * f);
        // correct = correct + 1.76606 * sin (2 * d - m + n - f);
        // correct = correct + 1.13466 * sin (2 * d - m + n + f);
        // correct = correct + .12897 * sin (2 * d - m + 2 * n - f);
        // correct = correct + .12387 * sin (2 * d - m + 2 * n + f);
        // correct = correct + .01211 * sin (2 * d - m + 3 * n + f);
        // correct = correct + .01127 * sin (2 * d - 5 * n - f);
        // correct = correct + .13381 * sin (2 * d - 4 * n - f);
        // correct = correct + .02496 * sin (2 * d - 4 * n + f);
        // correct = correct + 1.51564 * sin (2 * d - 3 * n - f);
        // correct = correct + .25408 * sin (2 * d - 3 * n + f);
        // correct = correct + .02045 * sin (2 * d - 2 * n - 3 * f);
        // correct = correct + 15.56635 * sin (2 * d - 2 * n - f);
        // correct = correct - 1.62443 * sin (2 * d - 2 * n + f);
        // correct = correct - .06561 * sin (2 * d - 2 * n + 3 * f);
        // correct = correct + .32907 * sin (2 * d - n - 3 * f);
        // correct = correct + 166.57528 * sin (2 * d - n - f);
        // correct = correct + 199.48515 * sin (2 * d - n + f);
        // correct = correct - .24484 * sin (2 * d - n + 3 * f);
        // correct = correct + 2.18637 * sin (2 * d - 3 * f);
        // correct = correct + 623.65783 * sin (2 * d - f);
        // correct = correct + 117.26161 * sin (2 * d + f);
        // correct = correct - .14453 * sin (2 * d + 3 * f);
        // correct = correct - .29116 * sin (2 * d + n - 3 * f);
        // correct = correct + 33.35743 * sin (2 * d + n - f);
        // correct = correct + 15.12165 * sin (2 * d + n + f);
        // correct = correct - .03038 * sin (2 * d + n + 3 * f);
        // correct = correct + 2.14618 * sin (2 * d + 2 * n - f);
        // correct = correct + 1.51976 * sin (2 * d + 2 * n + f);
        // correct = correct + .14642 * sin (2 * d + 3 * n - f);
        // correct = correct + .13795 * sin (2 * d + 3 * n + f);
        // correct = correct + .01027 * sin (2 * d + 4 * n - f);
        // correct = correct + .01186 * sin (2 * d + 4 * n + f);
        // correct = correct + .01818 * sin (2 * d + m - 3 * n - f);
        // correct = correct + .07913 * sin (2 * d + m - 2 * n - f);
        // correct = correct + .05429 * sin (2 * d + m - 2 * n + f);
        // correct = correct - .79105 * sin (2 * d + m - n - f);
        // correct = correct - 1.31788 * sin (2 * d + m - n + f);
        // correct = correct - .05457 * sin (2 * d + m - 3 * f);
        // correct = correct - 12.0947 * sin (2 * d + m - f);
        // correct = correct - 1.26433 * sin (2 * d + m + f);
        // correct = correct - .82275 * sin (2 * d + m + n - f);
        // correct = correct - .23702 * sin (2 * d + m + n + f);
        // correct = correct - .06283 * sin (2 * d + m + 2 * n - f);
        // correct = correct - .03142 * sin (2 * d + m + 2 * n + f);
        // correct = correct - .01262 * sin (2 * d + 2 * m - 2 * n - f);
        // correct = correct - .10535 * sin (2 * d + 2 * m - n - f);
        // correct = correct - .1133 * sin (2 * d + 2 * m - n + f);
        // correct = correct - .13415 * sin (2 * d + 2 * m - f);
        // correct = correct - .01482 * sin (2 * d + 2 * m + f);
        // correct = correct - .02104 * sin (3 * d - m - n - f);
        // correct = correct - .01356 * sin (3 * d - m - n + f);
        // correct = correct - .02572 * sin (3 * d - m - f);
        // correct = correct - .03941 * sin (3 * d - 2 * n - f);
        // correct = correct - .04852 * sin (3 * d - 2 * n + f);
        // correct = correct - .30517 * sin (3 * d - n - f);
        // correct = correct - .20593 * sin (3 * d - n + f);
        // correct = correct - .01009 * sin (3 * d - 3 * f);
        // correct = correct - .35183 * sin (3 * d - f);
        // correct = correct - .0284 * sin (3 * d + f);
        // correct = correct - .03611 * sin (3 * d + n - f);
        // correct = correct + .01321 * sin (3 * d + m - n - f);
        // correct = correct + .02083 * sin (3 * d + m - n + f);
        // correct = correct + .03436 * sin (3 * d + m - f);
        // correct = correct + .01351 * sin (3 * d + m + f);
        // correct = correct + .0123 * sin (4 * d - 2 * m - 2 * n + f);
        // correct = correct + .03462 * sin (4 * d - 2 * m - n - f);
        // correct = correct + .0238 * sin (4 * d - 2 * m - n + f);
        // correct = correct + .02899 * sin (4 * d - 2 * m - f);
        // correct = correct + .0127 * sin (4 * d - 2 * m + f);
        // correct = correct + .05251 * sin (4 * d - m - 2 * n - f);
        // correct = correct + .21376 * sin (4 * d - m - 2 * n + f);
        // correct = correct + .5958 * sin (4 * d - m - n - f);
        // correct = correct + .33882 * sin (4 * d - m - n + f);
        // correct = correct + .41496 * sin (4 * d - m - f);
        // correct = correct + .15791 * sin (4 * d - m + f);
        // correct = correct + .05686 * sin (4 * d - m + n - f);
        // correct = correct + .03009 * sin (4 * d - m + n + f);
        // correct = correct + .02174 * sin (4 * d - 3 * n + f);
        // correct = correct + .63371 * sin (4 * d - 2 * n - f);
        // correct = correct + 2.41389 * sin (4 * d - 2 * n + f);
        // correct = correct + 6.57962 * sin (4 * d - n - f);
        // correct = correct + 2.9985 * sin (4 * d - n + f);
        // correct = correct + .06257 * sin (4 * d - 3 * f);
        // correct = correct + 3.67449 * sin (4 * d - f);
        // correct = correct + 1.19188 * sin (4 * d + f);
        // correct = correct + .47338 * sin (4 * d + n - f);
        // correct = correct + .21259 * sin (4 * d + n + f);
        // correct = correct + .04834 * sin (4 * d + 2 * n - f);
        // correct = correct + .02828 * sin (4 * d + 2 * n + f);
        // correct = correct - .02957 * sin (4 * d + m - 2 * n + f);
        // correct = correct - .17191 * sin (4 * d + m - n - f);
        // correct = correct - .05097 * sin (4 * d + m - n + f);
        // correct = correct - .11308 * sin (4 * d + m - f);
        // correct = correct - .02549 * sin (4 * d + m + f);
        // correct = correct - .01692 * sin (4 * d + m + n - f);
        // correct = correct - .01049 * sin (5 * d - n - f);
        // correct = correct + .01091 * sin (6 * d - m - 2 * n - f);
        // correct = correct + .01486 * sin (6 * d - m - n - f);
        // correct = correct + .03118 * sin (6 * d - 3 * n + f);
        // correct = correct + .08096 * sin (6 * d - 2 * n - f);
        // correct = correct + .05963 * sin (6 * d - 2 * n + f);
        // correct = correct + .09403 * sin (6 * d - n - f);
        // correct = correct + .04217 * sin (6 * d - n + f);
        // correct = correct + .03674 * sin (6 * d - f);
        // correct = correct + .01465 * sin (6 * d + f);
        //
        // correct = correct + 8.045 * sin (dzeta_lune + 180 / r2d);
        // correct = correct + 0.416 * sin (dzeta_lune + petit_l + 180 / r2d);
        // correct = correct + 0.456 * sin (dzeta_lune - petit_l);
        // correct = correct + 0.326 * sin (dzeta_lune - 2 * f);
        //
        // correct = correct + 0.63 * sin (18 * venus - 16 * longitude_moyenne_terre - petit_l + f + 26.54 / r2d);
        // correct = correct + 0.63 * sin (18 * venus - 16 * longitude_moyenne_terre-petit_l - f + 26.54 / r2d);
        // correct = correct + 0.14 * sin (longitude_moyenne_terre + d + 291.98 / r2d);
        // correct = correct + 0.07 * sin (18 * venus - 16 * longitude_moyenne_terre - 2 * petit_l - f + 26.54 / r2d);
        // correct = correct + 0.067 * sin (18 * venus - 16 * longitude_moyenne_terre + f + 26.54 / r2d);
        // correct = correct + 0.067 * sin (5 * venus - 6 * longitude_moyenne_terre + 2 * d - f + 272.3 / r2d);
        //
        // correct = correct + 1.375 * sin (longitude_moyenne_terre + d + 275.13 / r2d);
        // correct = correct + 0.078 * sin (longitude_moyenne_terre + d - petit_l + 95.13 / r2d);
        //
        // correct = correct - 0.00001754 * 3600 / r2d * sin(183.3 / r2d + 483202 / r2d * T_2000);
        //
        // latitude = correct / 3600 / r2d;
        // b = latitude;
        // latitude_lune = b;
        //
        // b1 = abs(b * r2d);
        // d1 = ((b1 - floor(b1)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // if (b < 0) {signe = "-";}
        // else {signe = "+";}
        //
        // document.calc.latlune.value = signe + floor(b1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        //
        // p = 385000.56 - 3.14837 * cos(2 * f) + 79.66183 * cos(n - 2 * f);
        // p = p - 20905.32206 * cos(n) - 0.10326 * cos(n + 2 * f);
        // p = p - 4.42124 * cos(2 * n - 2 * f) - 569.92332 * cos(2 * n);
        // p = p - 23.21032 * cos(3 * n) - 1.11693 * cos(4 * n);
        // p = p - 0.42242 * cos(m - 3 * n) - 7.00293 * cos(m - 2 * n);
        // p = p - 129.62476 * cos(m - n) + 0.33465 * cos(m - n + 2 * f);
        // p = p - 0.18568 * cos(m - 2 * f) + 48.89 * cos(m);
        // p = p - 0.15803 * cos(m + 2 * f) - 0.2481 * cos(m + n - 2 * f);
        // p = p + 104.75896 * cos(m + n) + 5.75105 * cos(m + 2 * n);
        // p = p + 0.35509 * cos(m + 3 * n) - 0.13618 * cos(2 * m - 2 * n);
        // p = p - 2.11728 * cos(2 * m - n) + 1.06575 * cos(2 * m);
        // p = p + 1.16562 * cos(2 * m + n) + 0.1141 * cos(d - m - n);
        // p = p + 0.49757 * cos(d - m) + 0.10998 * cos(d - m + n);
        // p = p - 1.73852 * cos(d - 2 * n) - 8.37909 * cos(d - n);
        // p = p - 0.79564 * cos(d - 2 * f) + 108.74265 * cos(d);
        // p = p + 6.32199 * cos(d + n) + 0.37852 * cos(d + 2 * n);
        // p = p + 0.33226 * cos(d + m - 2 * n) + 0.85127 * cos(d + m - n);
        // p = p - 16.67533 * cos(d + m) - 0.93335 * cos(d + m + n);
        // p = p - 0.14808 * cos(2 * d - 3 * m - n) - 0.41076 * cos(2 * d - 3 * m);
        // p = p + 0.34304 * cos(2 * d - 2 * m - 2 * n) - 4.95049 * cos(2 * d - 2 * m - n);
        // p = p - 9.88519 * cos(2 * d - 2 * m) - 0.65758 * cos(2 * d - 2 * m + n);
        // p = p + 0.49506 * cos(2 * d - m - 3 * n) + 10.05654 * cos(2 * d - m - 2 * n);
        // p = p + 0.32336 * cos(2 * d - m - n - 2 * f) - 152.14314 * cos(2 * d - m - n);
        // p = p + 0.657 * cos(2 * d - m - 2 * f) - 204.59357 * cos(2 * d - m);
        // p = p + 0.20942 * cos(2 * d - m + n - 2 * f) - 12.83185 * cos(2 * d - m + n);
        // p = p - 0.84883 * cos(2 * d - m + 2 * n) + 0.77854 * cos(2 * d - 4 * n);
        // p = p + 14.40262 * cos(2 * d - 3 * n) + 0.47263 * cos(2 * d - 2 * n - 2 * f);
        // p = p + 246.15768 * cos(2 * d - 2 * n) + 0.77405 * cos(2 * d - 2 * n + 2 * f);
        // p = p + 8.7517 * cos(2 * d - n - 2 * f) - 3699.10468 * cos(2 * d - n);
        // p = p + 0.59633 * cos(2 * d - n + 2 * f) + 10.32129 * cos(2 * d - 2 * f);
        // p = p - 2955.9665 * cos(2 * d) + 4.13118 * cos(2 * d + n - 2 * f);
        // p = p - 170.73274 * cos(2 * d + n) + 0.28399 * cos(2 * d + 2 * n - 2 * f);
        // p = p - 10.44472 * cos(2 * d + 2 * n) - 0.66968 * cos(2 * d + 3 * n);
        // p = p + 0.16858 * cos(2 * d + m - 3 * n) + 0.14368 * cos(2 * d + m - 2 * n);
        // p = p + 24.20935 * cos(2 * d + m - n) - 0.13572 * cos(2 * d + m - 2 * f);
        // p = p + 30.82498 * cos(2 * d + m) + 2.6165 * cos(2 * d + m + n);
        // p = p + 0.21252 * cos(2 * d + m + 2 * n) - 0.10888 * cos(2 * d + 2 * m - 2 * n);
        // p = p + 2.3538 * cos(2 * d + 2 * m - n) + 0.14764 * cos(2 * d + 2 * m);
        // p = p + 0.2556 * cos(3 * d - m - n) - 0.15708 * cos(3 * d - m);
        // p = p + 0.86243 * cos(3 * d - 2 * n) + 3.25823 * cos(3 * d - n);
        // p = p + 0.20099 * cos(3 * d - 2 * f) - 1.41893 * cos(3 * d);
        // p = p - 0.21259 * cos(3 * d + m - n) - 0.10766 * cos(3 * d + m);
        // p = p - 0.10834 * cos(4 * d - 2 * m - 2 * n) - 0.27906 * cos(4 * d - 2 * m - n);
        // p = p - 0.12806 * cos(4 * d - 2 * m) - 1.897 * cos(4 * d - m - 2 * n);
        // p = p - 3.95812 * cos(4 * d - m - n) - 1.57145 * cos(4 * d - m);
        // p = p - 0.20286 * cos(4 * d - m + n) - 0.51423 * cos(4 * d - 3 * n);
        // p = p - 21.63627 * cos(4 * d - 2 * n) - 0.32176 * cos(4 * d - n - 2 * f);
        // p = p - 34.78245 * cos(4 * d - n) - 0.50793 * cos(4 * d - 2 * f);
        // p = p - 11.64993 * cos(4 * d) - 1.42255 * cos(4 * d + n);
        // p = p - 0.13922 * cos(4 * d + 2 * n) + 0.23696 * cos(4 * d + m - 2 * n);
        // p = p + 0.5788 * cos(4 * d + m - n) + 0.24453 * cos(4 * d + m);
        // p = p - 0.18316 * cos(6 * d - 3 * n) - 0.4225 * cos(6 * d - 2 * n);
        // p = p - 0.28663 * cos(6 * d - n);
        //
        // distance_terre = p;
        //
        // document.calc.distlune.value = floor(distance_terre * 1000 + 0.5) / 1000 + " km";
        //
        // p = sin(6378.136 / distance_terre) * r2d * 3600;
        // p = floor(p * 1000 + 0.5) / 1000;
        // p1 = floor(p / 60);
        //
        // document.calc.paralune.value = p1 + "'" + zero(floor((p - p1 * 60) * 1000 + 0.5) / 1000) + "''";
        //
        // diamapparent = atan(3476 / distance_terre) * r2d * 60
        // diamapparent1 = floor((diamapparent - floor(diamapparent)) * 60);
        // document.calc.diamapparent.value = floor(diamapparent) + "'" + floor(diamapparent1 * 100 + 0.5) / 100 + "''";
        //
        // asc = r2d / 15 * atan((cos(obliquite) * sin (longitude) -tan(latitude) * sin (obliquite)) / cos(longitude));
        // if (asc<0) {asc = asc + 24;}
        // if (cos(longitude) < 0) {asc = asc + 12;}
        // if (asc > 24) {asc = asc - 24;}
        //
        // declin = r2d * asin(sin(latitude) * cos(obliquite) + cos(latitude) * sin (obliquite) * sin(longitude));
        //
        // d1 = ((asc-floor(asc)) * 60);
        // d2 = ((d1 - floor(d1)) * 60);
        //
        // document.calc.alphalune.value = floor(asc) + "h" + zero(floor(d1)) + "m" + zero(floor(d2 * 100 + 0.5) / 100) + "s";
        // ADLune = asc * 15 / r2d
        //
        // d = abs(declin);
        // d1 = ((d - floor(d)) * 60);
        // d2= ((d1 - floor(d1)) * 60);
        //
        // if (declin < 0) {signe = "-"; DecLune = - d / r2d}
        // else {signe = "+"; DecLune = + d / r2d}
        //
        // document.calc.deltalune.value = signe + floor(d) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
        //
        // ascLune = asc
        // declinLune = declin
        //
        // d1 = acos(cos(latitude_lune) * cos(longitude_lune - longitude_soleil));
        // if (d1 < 0) {d1 = d1 + PI;}
        //
        // p = 180 - d1 * r2d - 0.1468 * sin(d1) * ((1 - 0.0549 * sin(anomalie_lune)) / (1 - 0.0167 * sin(anomalie_soleil)));
        // p = p / r2d;
        //
        // if ((longitude_lune < longitude_soleil && longitude_soleil - longitude_lune < PI) || (longitude_lune > longitude_soleil && longitude_lune - longitude_soleil > PI))
        // {elongation_lune = "Ouest";}
        // else {elongation_lune = "Est";}
        //
        // document.calc.elonglune.value = floor(d1 * r2d * 10 + 0.5) / 10 + "° " + elongation_lune;
        //
        // fraction_illuminee = 100 * floor(0.5 * (1 + cos(p)) * 1000 + 0.5) / 1000;
        // fraction_illuminee = floor(fraction_illuminee * 100 + 0.5) / 100;
        // elonglune=floor(d1 * r2d * 10 + 0.5) / 10
        //
        // document.calc.fractionlune.value = fraction_illuminee + "%"
        //
        // elong_lune = elongation_lune
        // fraction_illumineeLune = fraction_illuminee
        //
        // if (fraction_illumineeLune == 0){
        // commentLune = "Inobservable.";quandLune = " C'est la Nouvelle Lune"}
        // if (fraction_illumineeLune > 0 && fraction_illumineeLune < 7.1){
        // commentLune = "Inobservable.";quandLune = " C'est la Nouvelle Lune"}
        // if (fraction_illumineeLune > 7 && fraction_illumineeLune < 12.1){
        // commentLune = "Très fin croissant lunaire à tenter de voir"
        // if (elong_lune == "Ouest") {quandLune = " avant le lever du Soleil"}
        // if (elong_lune == "Est") {quandLune = " au coucher du Soleil"}}
        // if (fraction_illumineeLune > 12 && fraction_illumineeLune < 15.1){
        // commentLune = "Fin croissant lunaire à tenter de voir"
        // if (elong_lune == "Ouest") {quandLune = " avant le lever du Soleil"}
        // if (elong_lune == "Est") {quandLune = " au coucher du Soleil"}}
        // if (fraction_illumineeLune > 15 && fraction_illumineeLune < 30.1){
        // commentLune = "Croissant de Lune à voir"
        // if (elong_lune == "Ouest") {quandLune = " à l'aube"}
        // if (elong_lune == "Est") {quandLune = " au crépuscule"}}
        // if (fraction_illumineeLune > 30 && fraction_illumineeLune < 40.1){
        // commentLune = "Le gros croissant de Lune est"
        // if (elong_lune == "Ouest") {quandLune = " visible en fin de nuit"}
        // if (elong_lune == "Est") {quandLune = " visible en soirée"}}
        // if (fraction_illumineeLune > 40 && fraction_illumineeLune < 60.1){
        // commentLune = "La Lune est en phase de"
        // if (elong_lune == "Ouest") {quandLune = " Dernier Quartier"}
        // if (elong_lune == "Est") {quandLune = " Premier Quartier"}}
        // if (fraction_illumineeLune > 60 && fraction_illumineeLune < 95.1){
        // commentLune = "La Lune gibbeuse est"
        // if (elong_lune == "Ouest") {quandLune = " décroissante"}
        // if (elong_lune == "Est") {quandLune = " croissante"}}
        // if (fraction_illumineeLune > 95 && fraction_illumineeLune < 98.1){
        // commentLune = "La Lune est";
        // if (elong_lune == "Ouest") {quandLune = " encore bien Pleine"}
        // if (elong_lune == "Est") {quandLune = " presque Pleine"}}
        // if (fraction_illumineeLune > 98 && fraction_illumineeLune < 100.1){
        // commentLune = "Pleine Lune"; quandLune = ""}
        //
        // da_Lune_Merc = sin(DecLune) * sin(DecMerc) + cos(DecLune) * cos(DecMerc) * cos(ADLune - ADMerc)
        // da_Lune_Merc = acos(da_Lune_Merc) * r2d
        // if (da_Lune_Merc < 10){
        // da_Lune_Merc = ", à " + floor(da_Lune_Merc) + "°" + round(((da_Lune_Merc - floor(da_Lune_Merc)) * 60)) + "'" + " de Mercure" ;
        // }
        // else {da_Lune_Merc = ""}
        //
        // da_Lune_Ven = sin(DecLune) * sin(DecVen) + cos(DecLune) * cos(DecVen) * cos(ADLune - ADVen)
        // da_Lune_Ven = acos(da_Lune_Ven) * r2d
        // if (da_Lune_Ven < 10){
        // da_Lune_Ven = ", à " + floor(da_Lune_Ven) + "°" + round(((da_Lune_Ven - floor(da_Lune_Ven)) * 60)) + "'" + " de Vénus" ;
        // }
        // else {da_Lune_Ven = ""}
        //
        // da_Lune_Mars = sin(DecLune) * sin(DecMars) + cos(DecLune) * cos(DecMars) * cos(ADLune - ADMars)
        // da_Lune_Mars = acos(da_Lune_Mars) * r2d
        // if (da_Lune_Mars < 10){
        // da_Lune_Mars = ", à " + floor(da_Lune_Mars) + "°" + round(((da_Lune_Mars - floor(da_Lune_Mars)) * 60)) + "'" + " de Mars" ;
        // }
        // else {da_Lune_Mars = ""}
        //
        // da_Lune_Jup = sin(DecLune) * sin(DecJup) + cos(DecLune) * cos(DecJup) * cos(ADLune - ADJup)
        // da_Lune_Jup = acos(da_Lune_Jup) * r2d
        // if (da_Lune_Jup < 10){
        // da_Lune_Jup = ", à " + floor(da_Lune_Jup) + "°" + round(((da_Lune_Jup - floor(da_Lune_Jup)) * 60)) + "'" + " de Jupiter" ;
        // }
        // else {da_Lune_Jup = ""}
        //
        // da_Lune_Sat = sin(DecLune) * sin(DecSat) + cos(DecLune) * cos(DecSat) * cos(ADLune - ADSat)
        // da_Lune_Sat = acos(da_Lune_Sat) * r2d
        // if (da_Lune_Sat < 10){
        // da_Lune_Sat = ", à " + floor(da_Lune_Sat) + "°" + round(((da_Lune_Sat - floor(da_Lune_Sat)) * 60)) + "'" + " de Saturne" ;
        // }
        // else {da_Lune_Sat = ""}
        //
        // da_Lune_Uran = sin(DecLune) * sin(DecUran) + cos(DecLune) * cos(DecUran) * cos(ADLune - ADUran)
        // da_Lune_Uran = acos(da_Lune_Uran) * r2d
        // if (da_Lune_Uran < 10){
        // da_Lune_Uran = ", à " + floor(da_Lune_Uran) + "°" + round(((da_Lune_Uran - floor(da_Lune_Uran)) * 60)) + "'" + " de Uranus" ;
        // }
        // else {da_Lune_Uran = ""}
        //
        // da_Lune_Nept = sin(DecLune) * sin(DecNept) + cos(DecLune) * cos(DecNept) * cos(ADLune - ADNept)
        // da_Lune_Nept = acos(da_Lune_Nept) * r2d
        // if (da_Lune_Nept < 10){
        // da_Lune_Nept = ", à " + floor(da_Lune_Nept) + "°" + round(((da_Lune_Nept - floor(da_Lune_Nept)) * 60)) + "'" + " de Neptune" ;
        // }
        // else {da_Lune_Nept = ""}
        //
        // da_Lune_Pluton = sin(DecLune) * sin(DecPluton) + cos(DecLune) * cos(DecPluton) * cos(ADLune - ADPluton)
        // da_Lune_Pluton = acos(da_Lune_Pluton) * r2d
        // if (da_Lune_Pluton < 10){
        // da_Lune_Pluton = ", à " + floor(da_Lune_Pluton) + "°" + round(((da_Lune_Pluton - floor(da_Lune_Pluton)) * 60)) + "'" + " de Pluton" ;
        // }
        // else {da_Lune_Pluton = ""}
        //
        // document.calc.visibLune.value = commentLune + "" + quandLune + "" + da_Lune_Merc + "" + da_Lune_Ven + "" + da_Lune_Mars + "" + da_Lune_Jup + "" + da_Lune_Sat + "" + da_Lune_Uran + "" +
        // da_Lune_Nept + "" + da_Lune_Pluton;
        //
        // sepanglu = new Array()
        // sepangme = new Array()
        // sepangve = new Array()
        // sepangma = new Array()
        // sepangju = new Array()
        // sepangsa = new Array()
        // sepangur = new Array()
        // sepangne = new Array()
        // sepangpl = new Array()
        // sepangso = new Array()
        //
        // AscDt = new Array(10)
        // Declinaison = new Array(10)
        //
        // AscDt[0] = ADLune
        // Declinaison[0] = DecLune
        // AscDt[1] = ADMerc
        // Declinaison[1] = DecMerc
        // AscDt[2] = ADVen
        // Declinaison[2] = DecVen
        // AscDt[3] = ADMars
        // Declinaison[3] = DecMars
        // AscDt[4] = ADJup
        // Declinaison[4] = DecJup
        // AscDt[5] = ADSat
        // Declinaison[5] = DecSat
        // AscDt[6] = ADUran
        // Declinaison[6] = DecUran
        // AscDt[7] = ADNept
        // Declinaison[7] = DecNept
        // AscDt[8] = ADPluton
        // Declinaison[8] = DecPluton
        // AscDt[9] = ADSoleil
        // Declinaison[9] = DecSoleil
        //
        //
        // indx = floor(random() * AscDt.length);
        //
        // for (indx = 0; indx < 10; indx++){
        // ax = 0
        // sepanglu [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // ax = ax+1
        // sepangme [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // ax = ax+1
        // sepangve [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // ax = ax+1
        // sepangma [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // ax = ax+1
        // sepangju [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // ax = ax+1
        // sepangsa [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // ax = ax+1
        // sepangur [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // ax = ax+1
        // sepangne [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // ax = ax+1
        // sepangpl [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // ax = ax+1
        // sepangso [indx] = aff(Math.floor(100 * r2d * acos(Math.sin(Declinaison[indx]) * Math.sin(Declinaison[ax]) + Math.cos(Declinaison[indx]) * Math.cos(Declinaison[ax]) * Math.cos(AscDt[indx] -
        // AscDt[ax]))) / 100);
        // }
        //
        // document.calc.lume.value = sepanglu[1]
        // document.calc.luve.value = sepanglu[2]
        // document.calc.luma.value = sepanglu[3]
        // document.calc.luju.value = sepanglu[4]
        // document.calc.lusa.value = sepanglu[5]
        // document.calc.luur.value = sepanglu[6]
        // document.calc.lune.value = sepanglu[7]
        // document.calc.lupl.value = sepanglu[8]
        // document.calc.luso.value = sepanglu[9]
        //
        // document.calc.melu.value = sepangme[0]
        // document.calc.meve.value = sepangme[2]
        // document.calc.mema.value = sepangme[3]
        // document.calc.meju.value = sepangme[4]
        // document.calc.mesa.value = sepangme[5]
        // document.calc.meur.value = sepangme[6]
        // document.calc.mene.value = sepangme[7]
        // document.calc.mepl.value = sepangme[8]
        // document.calc.meso.value = sepangme[9]
        //
        // document.calc.velu.value = sepangve[0]
        // document.calc.veme.value = sepangve[1]
        // document.calc.vema.value = sepangve[3]
        // document.calc.veju.value = sepangve[4]
        // document.calc.vesa.value = sepangve[5]
        // document.calc.veur.value = sepangve[6]
        // document.calc.vene.value = sepangve[7]
        // document.calc.vepl.value = sepangve[8]
        // document.calc.veso.value = sepangve[9]
        //
        // document.calc.malu.value = sepangma[0]
        // document.calc.mame.value = sepangma[1]
        // document.calc.mave.value = sepangma[2]
        // document.calc.maju.value = sepangma[4]
        // document.calc.masa.value = sepangma[5]
        // document.calc.maur.value = sepangma[6]
        // document.calc.mane.value = sepangma[7]
        // document.calc.mapl.value = sepangma[8]
        // document.calc.maso.value = sepangma[9]
        //
        // document.calc.julu.value = sepangju[0]
        // document.calc.jume.value = sepangju[1]
        // document.calc.juve.value = sepangju[2]
        // document.calc.juma.value = sepangju[3]
        // document.calc.jusa.value = sepangju[5]
        // document.calc.juur.value = sepangju[6]
        // document.calc.june.value = sepangju[7]
        // document.calc.jupl.value = sepangju[8]
        // document.calc.juso.value = sepangju[9]
        //
        // document.calc.salu.value = sepangsa[0]
        // document.calc.same.value = sepangsa[1]
        // document.calc.save.value = sepangsa[2]
        // document.calc.sama.value = sepangsa[3]
        // document.calc.saju.value = sepangsa[4]
        // document.calc.saur.value = sepangsa[6]
        // document.calc.sane.value = sepangsa[7]
        // document.calc.sapl.value = sepangsa[8]
        // document.calc.saso.value = sepangsa[9]
        //
        // document.calc.urlu.value = sepangur[0]
        // document.calc.urme.value = sepangur[1]
        // document.calc.urve.value = sepangur[2]
        // document.calc.urma.value = sepangur[3]
        // document.calc.urju.value = sepangur[4]
        // document.calc.ursa.value = sepangur[5]
        // document.calc.urne.value = sepangur[7]
        // document.calc.urpl.value = sepangur[8]
        // document.calc.urso.value = sepangur[9]
        //
        // document.calc.nelu.value = sepangne[0]
        // document.calc.neme.value = sepangne[1]
        // document.calc.neve.value = sepangne[2]
        // document.calc.nema.value = sepangne[3]
        // document.calc.neju.value = sepangne[4]
        // document.calc.nesa.value = sepangne[5]
        // document.calc.neur.value = sepangne[6]
        // document.calc.nepl.value = sepangne[8]
        // document.calc.neso.value = sepangne[9]
        //
        // document.calc.pllu.value = sepangpl[0]
        // document.calc.plme.value = sepangpl[1]
        // document.calc.plve.value = sepangpl[2]
        // document.calc.plma.value = sepangpl[3]
        // document.calc.plju.value = sepangpl[4]
        // document.calc.plsa.value = sepangpl[5]
        // document.calc.plur.value = sepangpl[6]
        // document.calc.plne.value = sepangpl[7]
        // document.calc.plso.value = sepangpl[9]
        //
        // document.calc.solu.value = sepangso[0]
        // document.calc.some.value = sepangso[1]
        // document.calc.sove.value = sepangso[2]
        // document.calc.soma.value = sepangso[3]
        // document.calc.soju.value = sepangso[4]
        // document.calc.sosa.value = sepangso[5]
        // document.calc.sour.value = sepangso[6]
        // document.calc.sone.value = sepangso[7]
        // document.calc.sopl.value = sepangso[8]
        //
        // //--------------------------------------------- Lever - Coucher - Passage au Méridien du Soleil
        //
        // pi = 3.141593
        // dr = pi / 180
        // hr = pi / 12
        //
        // efr0 = -0.009890199 //effet de réfraction sur la ligne d'horizon - valeur adoptée de 0°34'
        // efr1 = -0.004654211 //effet de réfraction sur la ligne d'horizon - demi-diamètre de 0°16' pour le Soleil
        // ht = efr0 + efr1 //effet de réfraction sur la ligne d'horizon pour le Soleil
        // parLune = 0.016580628 // parallaxe (pour la Lune)
        // dmLune = 0.004654211 // demi-diamètre lunaire
        // ht0 = parLune - efr0 - dmLune
        //
        // htc = - 6 * dr // hauteur crépuscule civil
        // htn = - 12 * dr // hauteur crépuscule nautique
        // hta = - 18 * dr // hauteur crépuscule astronomique
        //
        // //--------------- "Longitude (en degré, positive à l'ouest)";
        //
        // if (nst2 == 1) {lonObs = -(londeg + (lonmin / 60)) * dr}
        // if (nst2 == 2) {lonObs = (londeg + (lonmin / 60)) * dr}
        //
        // //--------------- "Latitude (en degré)";
        //
        // if (nst1 == 1){latObs = (latdeg + (latmin / 60)) * dr}
        // if (nst1 == 2) {latObs = -(latdeg + (latmin / 60)) * dr}
        //
        // //--------------- Heure TU du milieu de la journée
        //
        // h = 12 + lonObs / hr
        //
        // //--------------- Coordonnées rectangulaires du soleil dans le repère équatorial
        //
        // xe = cos(Lov)
        // ye = cos(obliquite) * sin(Lov)
        // ze = sin(obliquite) * sin(Lov)
        //
        // //--------------- Rotation de l'angle r autour de l'axe z
        //
        // rx = cos(Lo) * xe + sin(Lo) * ye
        // ry = -sin(Lo) * xe + cos(Lo) * ye
        // xf = rx
        // yf = ry
        //
        // et = atan(yf / xf)
        // dc = atan(ze / sqrt(1 - ze * ze))
        // decl = atan(ze / sqrt(1 - ze * ze)) / dr
        //
        // //--------------- Heure de passage au méridien
        //
        // pm = h + et / hr
        // merh = floor(pm)
        // pm = 60 * (pm - merh)
        // merm = floor(pm)
        // pm = floor(60 * (pm - merm))
        // document.calc.mer.value = notn(zero(merh)) + "h" + notn(zero(merm)) + "m";
        //
        // //--------------- Angle Horaire au lever et au coucher, Lever et Coucher du Soleil,
        // //--------------- AH au Crépuscule civil, Crépuscule civil du matin et du soir,
        // //--------------- AH au Crépuscule nautique, Crépuscule nautique du matin et du soir,
        // //--------------- AH au Crépuscule astronomique, Crépuscule astronomique du matin et du soir
        //
        // hauteur = new Array(); levh = new Array(); levm = new Array(); couh = new Array(); coum = new Array()
        // hauteur[0] = ht
        // hauteur[1] = htc
        // hauteur[2] = htn
        // hauteur[3] = hta
        //
        // i = floor(random() * hauteur.length);
        //
        // for (i = 0; i < 4; i++)
        // {
        // cs = (sin(hauteur[i]) - sin(latObs) * sin(dc)) / cos(latObs) / cos(dc)
        // if (cs == 0) { ah = pi / 2}
        // {ah = atan(sqrt(1 - cs * cs) / cs)}
        // if (cs < 0){ah = ah + pi}
        // pm1 = h + (et - ah) / hr;
        // if (pm1 < 0){pm1 = pm1 + 24}
        // levh[i] = floor(pm1);
        // levm[i] = floor(60 * (pm1 - levh[i]));
        // pm2 = h + (et + ah) / hr;
        // if (pm2 > 24){pm2 = pm2 - 24}
        // couh[i] = floor(pm2);
        // coum[i] = floor(60 * (pm2 - couh[i]));
        // }
        //
        // document.calc.lev.value = notn(zero(levh[0])) + "h" + notn(zero(levm[0])) + "m";
        // document.calc.cou.value = notn(zero(couh[0])) + "h" + notn(zero(coum[0])) + "m";
        // document.calc.ccm.value = notn(zero(levh[1])) + "h" + notn(zero(levm[1])) + "m";
        // document.calc.ccs.value = notn(zero(couh[1])) + "h" + notn(zero(coum[1])) + "m";
        // document.calc.cnm.value = notn(zero(levh[2])) + "h" + notn(zero(levm[2])) + "m";
        // document.calc.cns.value = notn(zero(couh[2])) + "h" + notn(zero(coum[2])) + "m";
        // document.calc.cam.value = notn(zero(levh[3])) + "h" + notn(zero(levm[3])) + "m";
        // document.calc.cas.value = notn(zero(couh[3])) + "h" + notn(zero(coum[3])) + "m";
        //
        // //--------------- Hauteur - Azimut
        //
        // RA = new Array(9)
        // Dec = new Array(9)
        // RA[0] = ascLune
        // Dec[0] = declinLune
        // RA[1] = ascMerc
        // Dec[1] = declinMerc
        // RA[2] = ascVen
        // Dec[2] = declinVen
        // RA[3] = ascMars
        // Dec[3] = declinMars
        // RA[4] = ascJup
        // Dec[4] = declinJup
        // RA[5] = ascSat
        // Dec[5] = declinSat
        // RA[6] = ascUran
        // Dec[6] = declinUran
        // RA[7] = ascNept
        // Dec[7] = declinNept
        // RA[8] = ascPluton
        // Dec[8] = declinPluton
        // RA[9] = ascSoleil
        // Dec[9] = declinSoleil
        //
        //
        // ind = floor(random() * RA.length);
        //
        // GST = ts1
        // HEURE = heure;
        // MINUTE = minute;
        // d2r = PI / 180
        // r2d = 180 / PI;
        //
        // lng = -lonObs / d2r
        // lat = latObs / d2r
        //
        // alt = new Array()
        // az = new Array()
        // num = new Array()
        // denom = new Array()
        //
        // for (ind = 0; ind < 10; ind++){
        //
        // HA = 15 * (GST + HEURE + (MINUTE / 60) - RA[ind]) + lng;
        // alt[ind] = r2d * asin(sin(Dec[ind] * d2r) * sin(lat * d2r) + cos(Dec[ind] * d2r) * cos(HA * d2r) * cos(lat * d2r));
        // alt[ind] = floor(alt[ind]* 10) / 10
        //
        // if (alt[ind] > 0){
        //
        // if (nst1 == 1)
        // {
        // num[ind] = sin(HA * d2r); denom[ind] = (cos(HA * d2r) * sin(lat * d2r) - tan(Dec[ind] * d2r) * cos(lat * d2r));
        // azimut = num[ind] / denom[ind];
        // az[ind] = 180 + atan(azimut) * r2d;
        // if (denom[ind] < 0) az[ind] = az[ind] + 180;
        // }
        // else
        // {
        // num[ind] = sin( - HA * d2r); denom[ind] = (cos( - HA * d2r) * sin( - lat * d2r) - tan( - Dec[ind] * d2r) * cos( - lat * d2r));
        // azimut = num[ind] / denom[ind];
        // az[ind] = 180 - atan( - azimut) * r2d;
        // if (denom[ind] > 0) az[ind] = az[ind] - 180;
        // }
        //
        // if (azimut < 0) az[ind] += 360;
        // if (az[ind] > 360) {az[ind] -= 360};
        //
        // az[ind] = floor(az[ind]) + "°" + floor(60 * (az[ind] - floor(az[ind]))) + "'"
        //
        // }
        // else az[ind] = "---"
        //
        // alt[ind] = floor(alt[ind]) + "°" + floor(60 * (alt[ind] - floor(alt[ind]))) + "'"
        // }
        //
        // document.calc.altLune.value = alt[0]
        // document.calc.azLune.value = az[0]
        // document.calc.altMerc.value = alt[1]
        // document.calc.azMerc.value = az[1]
        // document.calc.altVen.value = alt[2]
        // document.calc.azVen.value = az[2]
        // document.calc.altMars.value = alt[3]
        // document.calc.azMars.value = az[3]
        // document.calc.altJup.value = alt[4]
        // document.calc.azJup.value = az[4]
        // document.calc.altSat.value = alt[5]
        // document.calc.azSat.value = az[5]
        // document.calc.altUran.value = alt[6]
        // document.calc.azUran.value = az[6]
        // document.calc.altNept.value = alt[7]
        // document.calc.azNept.value = az[7]
        // document.calc.altPluton.value = alt[8]
        // document.calc.azPluton.value = az[8]
        // document.calc.altSoleil.value = alt[9]
        // document.calc.azSoleil.value = az[9]
        //
        // //--------------- Lever et Coucher des Planètes
        //
        // indh = floor(random() * RA.length);
        //
        // levhP = new Array(); levmP = new Array(); couhP = new Array(); coumP = new Array(); PassMh = new Array(); PassMm = new Array();
        //
        //
        // for (indh = 0; indh < 9; indh++){
        // if (indh==0) {csP = (sin(ht0) - sin(latObs) * sin(Dec[indh]*d2r)) / cos(latObs) / cos(Dec[indh]*d2r)}
        // else {
        // csP = (sin(efr0) - sin(latObs) * sin(Dec[indh]*d2r)) / cos(latObs) / cos(Dec[indh]*d2r)
        // }
        // AH = Math.acos(csP) / dr / 15
        //
        // //------ Lever
        //
        // TSl = (RA[indh] - AH)
        // T1l = TSl - (-lonObs / dr / 15)
        // Ttl = T1l - ts1
        // if (Ttl < 0) {Ttl += 24}
        // Ttl = Ttl / 1.002737908
        // if (Ttl < 0) {Ttl += 24}
        // if (Ttl > 24) {Ttl -= 24}
        // levhP[indh] = floor(Ttl);
        // levmP[indh] = floor(60 * (Ttl - levhP[indh]));
        //
        // //------ Coucher
        //
        // TSc = (RA[indh] + AH)
        // T1c = TSc - (-lonObs / dr / 15)
        // Ttc = T1c - ts1
        // if (Ttc < 0) {Ttc += 24}
        // Ttc = Ttc / 1.002737908
        // if (Ttc < 0) {Ttc += 24}
        // if (Ttc > 24) {Ttc -= 24}
        // couhP[indh] = floor(Ttc);
        // coumP[indh] = floor(60 * (Ttc - couhP[indh]));
        //
        // //------ Passage au Méridien
        // if (Ttc < Ttl) {Ttc += 24}
        // PassM = (Ttl + Ttc) / 2
        // if (PassM > 24) {PassM -= 24}
        //
        // PassMh[indh] = floor(PassM);
        // PassMm[indh] = floor(60 * (PassM - PassMh[indh]));
        // }
        //
        // document.calc.casMl.value = notn(zero(levhP[1])) + "h" + notn(zero(levmP[1])) + "m";
        // document.calc.pasMl.value = notn(zero(PassMh[1])) + "h" + notn(zero(PassMm[1])) + "m";
        // document.calc.casMc.value = notn(zero(couhP[1])) + "h" + notn(zero(coumP[1])) + "m";
        //
        // document.calc.casVl.value = notn(zero(levhP[2])) + "h" + notn(zero(levmP[2])) + "m";
        // document.calc.pasVl.value = notn(zero(PassMh[2])) + "h" + notn(zero(PassMm[2])) + "m";
        // document.calc.casVc.value = notn(zero(couhP[2])) + "h" + notn(zero(coumP[2])) + "m";
        //
        // document.calc.casMAl.value = notn(zero(levhP[3])) + "h" + notn(zero(levmP[3])) + "m";
        // document.calc.pasMAl.value = notn(zero(PassMh[3])) + "h" + notn(zero(PassMm[3])) + "m";
        // document.calc.casMAc.value = notn(zero(couhP[3])) + "h" + notn(zero(coumP[3])) + "m";
        //
        // document.calc.casJl.value = notn(zero(levhP[4])) + "h" + notn(zero(levmP[4])) + "m";
        // document.calc.pasJl.value = notn(zero(PassMh[4])) + "h" + notn(zero(PassMm[4])) + "m";
        // document.calc.casJc.value = notn(zero(couhP[4])) + "h" + notn(zero(coumP[4])) + "m";
        //
        // document.calc.casSl.value = notn(zero(levhP[5])) + "h" + notn(zero(levmP[5])) + "m";
        // document.calc.pasSl.value = notn(zero(PassMh[5])) + "h" + notn(zero(PassMm[5])) + "m";
        // document.calc.casSc.value = notn(zero(couhP[5])) + "h" + notn(zero(coumP[5])) + "m";
        //
        // document.calc.casUl.value = notn(zero(levhP[6])) + "h" + notn(zero(levmP[6])) + "m";
        // document.calc.pasUl.value = notn(zero(PassMh[6])) + "h" + notn(zero(PassMm[6])) + "m";
        // document.calc.casUc.value = notn(zero(couhP[6])) + "h" + notn(zero(coumP[6])) + "m";
        //
        // document.calc.casNl.value = notn(zero(levhP[7])) + "h" + notn(zero(levmP[7])) + "m";
        // document.calc.pasNl.value = notn(zero(PassMh[7])) + "h" + notn(zero(PassMm[7])) + "m";
        // document.calc.casNc.value = notn(zero(couhP[7])) + "h" + notn(zero(coumP[7])) + "m";
        //
        // document.calc.casPl.value = notn(zero(levhP[8])) + "h" + notn(zero(levmP[8])) + "m";
        // document.calc.pasPl.value = notn(zero(PassMh[8])) + "h" + notn(zero(PassMm[8])) + "m";
        // document.calc.casPc.value = notn(zero(couhP[8])) + "h" + notn(zero(coumP[8])) + "m";
        //
        // }
        //
        // gen()
        // compute()
        // }
        //
        // // Calcul des phases de la Lune
        // public void gen(){
        //
        // with (document.calc) {
        // jour = parseInt(day.value);
        // mois = parseInt(month.value);
        // annee = parseInt(year.value);
        // heure = parseInt(hour.value);
        // minute = parseInt(minut.value);
        // seconde = parseInt(second.value);
        // }
        //
        // window.status = "Calcul en cours...";
        //
        // // Numéro du jour de l'année
        // nj = Math.floor(275 * mois / 9) - 2 * Math.floor((mois + 9) / 12) + jour - 30;
        // fnj = nj / 365;
        //
        // pl = new Array(4)
        // ph = new Array(4)
        // yp= new Array(4)
        // yph= new Array(4)
        //
        // ph[0] = "Nouvelle Lune"
        // ph[1] = "Premier Quartier"
        // ph[2] = "Pleine Lune"
        // ph[3] = "Dernier Quartier"
        //
        // // Calcul des dates des phases
        // // note : si l'écart entre l'instant de la phase et le moment du calcul est supérieur à 15 jours, le calcul est effectué en se décalant de 15 jours afin d'afficher aussi bien les phases
        // précédentes que les phases suivantes par rapport à la date du jour
        //
        // n = 0
        // for (i = 0; i < 5; i++){
        // nextph = phase_lune(n);
        //// if (pt - jj > 15) {fnj -= 0.082191781; pl[i] = phase_lune(n); countA = phase_lune(0); fnj += 0.082191781;}
        // if (pt - jj > 15) {fnj -= (29.53058868/365); pl[i] = phase_lune(n); countA = phase_lune(0); fnj += (29.53058868/365);}
        // else {pl[i] = phase_lune(n); countA = phase_lune(0);}
        // n = n + 0.25
        // }
        //
        // // Construction du tableau
        // yp[0] = pl[0]; yp[1] = pl[1]; yp[2] = pl[2]; yp[3] = pl[3];
        // yph[0] = ph[0]; yph[1] = ph[1]; yph[2] = ph[2]; yph[3] = ph[3];
        // permute()
        //
        // document.calc.dt0.value = yph[0];
        // document.calc.dt1.value = yph[1];
        // document.calc.dt2.value = yph[2];
        // document.calc.dt3.value = yph[3];
        //
        // document.calc.pl0.value = edate(yp[0]);
        // document.calc.pl1.value = edate(yp[1]);
        // document.calc.pl2.value = edate(yp[2]);
        // document.calc.pl3.value = edate(yp[3]);
        //
        // // calculate moon's age in days
        //
        // if (jj < phase_lune(0)) {ag = jj - countA}
        // else {countA = phase_lune(0); ag = jj - countA}
        //
        // if (ag < 3 ){
        // agh = (Math.floor(ag * 24));
        // agm = Math.floor((ag*24 - Math.floor(ag * 24)) * 60);
        // document.calc.age.value = agh + "h "+ zero(agm) + "mn ";}
        //
        // else {
        // agj = (Math.floor(ag))
        // agh = Math.floor((ag * 24) - (agj * 24));
        // agm = Math.floor((((ag * 24) - (agj * 24)) - agh) * 60);
        // document.calc.age.value = agj + "j " + zero(agh) + "h "+ zero(agm) + "mn ";}
        //
        // window.status = "Terminé.";
        // }
        //
        // /*
        // * JYEAR -- Convert Julian date to year, month, day, which are
        // * returned as an Array.
        // */
        //
        // public void jyear(td) {
        // var z, f, a, alpha, b, c, d, e, mm;
        //
        // td += 0.5;
        // z = Math.floor(td);
        // f = td - z;
        //
        // if (z < 2299161.0) {
        // a = z;
        // } else {
        // alpha = Math.floor((z - 1867216.25) / 36524.25);
        // a = z + 1 + alpha - Math.floor(alpha / 4);
        // }
        //
        // b = a + 1524;
        // c = Math.floor((b - 122.1) / 365.25);
        // d = Math.floor(365.25 * c);
        // e = Math.floor((b - d) / 30.6001);
        // mm = Math.floor((e < 14) ? (e - 1) : (e - 13));
        //
        // return new Array(
        // Math.floor((mm > 2) ? (c - 4716) : (c - 4715)),
        // mm,
        // Math.floor(b - d - Math.floor(30.6001 * e) + f)
        // );
        // }
        //
        // /*
        // * JHMS -- Convert Julian time to hour, minutes, and seconds,
        // * returned as a three-element array.
        // */
        //
        // public void jhms(j) {
        // var ij;
        //
        // j += 0.5; /* Astronomical to civil */
        // ij = (j - Math.floor(j)) * 86400.0;
        // return new Array(
        // Math.floor(ij / 3600),
        // Math.floor((ij / 60) % 60),
        // Math.floor(ij % 60));
        // }
        //
        // /* DTR -- Degrees to radians. */
        //
        // public double dtr(double xd)
        // {
        // return (d * Math.PI) / 180.0;
        // }
        //
        // /* EDATE -- Edit date and time to application specific format. */
        //
        // var Months = new Array("Janvier", "Février", "Mars", "Avril", "Mai", "Juin", "Juillet", "Août", "Septembre", "Octobre", "Novembre", "Décembre");
        //
        // public void edate(j) {
        // var date2, time;
        //
        // j += (30.0 / (24 * 60 * 60)); // Round to nearest minute
        // date2 = jyear(j);
        // time = jhms(j);
        //
        // return "Le " + (zero(date2[2], 2, " ")) + " " + Months[date2[1] - 1] + " à " +
        // (zero(time[0], 2, " ")) + "h" + (zero(time[1], 2, "0")) + " UTC ";
        // }
        //
        // public void permute()
        // {
        // if(yp[0] > yp[1])
        // {
        // yp[0] = pl[1];
        // yp[1] = pl[2];
        // yp[2] = pl[3];
        // yp[3] = pl[0];
        // yph[0] = ph[1];
        // yph[1] = ph[2];
        // yph[2] = ph[3];
        // yph[3] = ph[0];
        // }
        // if(yp[1] > yp[2])
        // {
        // yp[0] = pl[2];
        // yp[1] = pl[3];
        // yp[2] = pl[0];
        // yp[3] = pl[1];
        // yph[0] = ph[2];
        // yph[1] = ph[3];
        // yph[2] = ph[0];
        // yph[3] = ph[1];
        // }
        // if(yp[2] > yp[3])
        // {
        // yp[0] = pl[3];
        // yp[1] = pl[0];
        // yp[2] = pl[1];
        // yp[3] = pl[2];
        // yph[0] = ph[3];
        // yph[1] = ph[0];
        // yph[2] = ph[1];
        // yph[3] = ph[2];
        // }
        //
        // }
        //
        // public void phase_lune(phase) {
        // with (Math) {
        // var d2r = PI/180;
        //
        // // Les commentaires et les formules utilisées pour le calcul des phases de la Lune proviennent du livre de Jean MEEUS
        // // "Calculs Astronomiques à l'usage des amateurs" Edition SAF ISBN 2-901730-03-6
        // // Valeur du mois synodique
        // SynMonth = 29.53058868
        //
        // // k peut être déterminé de manière approchée par :
        // k = (annee + fnj - 1900) * 12.3685;
        //
        // // Astronomique en civil
        // k += 0.5;
        // k = floor(k);
        // k += phase;
        //
        // // t est le temps en siècles juliens à partir du 0.5 janvier 1900
        // t = (jj - 2415020) / 36525;
        //
        // // Une fois que la valeur correcte de k a été trouvée, t peut être calculé avec une précision suffisante par :
        // t = k / 1236.85;
        //
        // t2 = t * t;
        // t3 = t2 * t;
        //
        // // Le moment des phases moyennes de la Lune, affecté de l'aberration du Soleil est donné par :
        //
        // pt = 2415020.75933 + SynMonth * k + 0.0001178 * t2 - 0.000000155 * t3 + 0.00033 * sin((166.56 + 132.87 * t - 0.009173 * t2) * d2r);
        //
        // // Ces instants sont exprimés en Temps des Ephémérides (Jours Juliens des Ephémérides).
        // // Dans la formule ci-dessus, une valeur entière de k donne l'instant de la Nouvelle Lune,
        // // une valeur entière augmentée de 0.25 donne l'instant du Premier Quartier,
        // // 0.50 donne l'instant de la Pleine Lune, 0.75 donne l'instant du Dernier Quartier
        // // N'importe quelle autre valeur de k donne un résultat sans signification/
        // // Une valeur négative de k correspond à une phase antérieure à l'année 1900, une valeur
        // // positive à une phase postérieure au début de l'année 1900.
        //
        // // Anomalie moyenne du Soleil à l'instant jj
        // m = 359.2242 + 29.10535608 * k - 0.0000333 * t2 - 0.00000347 * t3;
        //
        // // Anomalie moyenne de la Lune
        // mprime = 306.0253 + 385.81691806 * k + 0.0107306 * t2 + 0.00001236 * t3;
        //
        // //Argument de la latitude de la Lune
        // f = 21.2964 + 390.67050646 * k - 0.0016528 * t2 - 0.00000239 * t3;
        //
        // // Pour obtenir l'instant de la phase vraie, il faut ajouter les corrections suivantes à l'instant de la phase moyenne de la Lune.
        // // Les coefficients suivants sont exprimés en fractions décimales de jour; des quantités plus faibles ont été négligées.
        // // Pour la Nouvelle Lune et la Pleine Lune :
        //
        // if (phase == 0||phase == 0.5) {
        // pt += (0.1734 - 0.000393 * t) * sin(m * d2r);
        // pt += 0.0021 * sin(2 * m * d2r);
        // pt -= 0.4068 * sin(mprime * d2r);
        // pt += 0.0161 * sin(2 * mprime * d2r);
        // pt -= 0.0004 * sin(3 * mprime * d2r);
        // pt += 0.0104 * sin(2 * f * d2r);
        // pt -= 0.0051 * sin((m + mprime) * d2r);
        // pt -= 0.0074 * sin((m - mprime) * d2r);
        // pt += 0.0004 * sin((2 * f + m) * d2r);
        // pt -= 0.0004 * sin((2 * f - m) * d2r);
        // pt -= 0.0006 * sin((2 * f + mprime) * d2r);
        // pt += 0.0010 * sin((2 * f - mprime) * d2r);
        // pt += 0.0005 * sin((m + 2 * mprime) * d2r);
        // }
        //
        // // Pour le Premier et le Dernier Quartier :
        //
        // if (phase == 0.25||phase == 0.75) {
        // pt += (0.1721 - 0.0004 * t) * sin(m * d2r);
        // pt += 0.0021 * sin(2 * m * d2r);
        // pt -= 0.6280 * sin(mprime * d2r);
        // pt += 0.0089 * sin(2 * mprime * d2r);
        // pt -= 0.0004 * sin(3 * mprime * d2r);
        // pt += 0.0079 * sin(2 * f * d2r);
        // pt -= 0.0119 * sin((m + mprime) * d2r);
        // pt -= 0.0047 * sin((m - mprime) * d2r);
        // pt += 0.0003 * sin((2 * f + m) * d2r);
        // pt -= 0.0004 * sin((2 * f - m) * d2r);
        // pt -= 0.0006 * sin((2 * f + mprime) * d2r);
        // pt += 0.0021 * sin((2 * f - mprime) * d2r);
        // pt += 0.0003 * sin((m + 2 * mprime) * d2r);
        // pt += 0.0004 * sin((m - 2 * mprime) * d2r);
        // pt -= 0.0003 * sin((2 * m + mprime) * d2r);
        // }
        //
        // // et, de plus,
        //
        // if (phase == 0.25) {pt += 0.0028 - 0.0004 * cos(m * d2r) + 0.0003 * cos(mprime * d2r);}
        // if (phase == 0.75) {pt += - 0.0028 + 0.0004 * cos(m * d2r) - 0.0003 * cos(mprime * d2r);}
        // return pt;
        // }
        // }
        //
        // // Calcul du Lever et coucher de la Lune pour le jour en cours
        // // basé d'après un programme en basic publié dans Sky & Telescope, July 1989, page 78
        // public void compute()
        // {
        // var lat = latObs/d2r
        // var lon = lng
        // document.calc.moonrise.value = "";
        // document.calc.moonset.value = "";
        // document.calc.moonmsg.value = "";
        // riseset(lat, lon);
        // }
        //
        // // calculate moonrise and moonset times
        // public void riseset( lat, lon )
        // {
        // var i, j, k;
        // var zone = Math.round(0/60); // UTC
        // var jd = julian_day() - 2451545; // Julian day relative to Jan 1.5, 2000
        //
        // var mp = new Array(3); // create a 3x3 array
        // for (i = 0; i < 3; i++)
        // {
        // mp[i] = new Array(3);
        // for (j = 0; j < 3; j++)
        // mp[i][j] = 0.0;
        // }
        // lon = lon/360;
        // var tz = zone/24;
        // var t0 = lst(lon, jd, tz); // local sidereal time
        // jd = jd + tz; // get moon position at start of day
        // for (k = 0; k < 3; k++)
        // {
        // moon(jd);
        // mp[k][0] = Sky[0];
        // mp[k][1] = Sky[1];
        // mp[k][2] = Sky[2];
        // jd = jd + 0.5;
        // }
        // if (mp[1][0] <= mp[0][0])
        // mp[1][0] = mp[1][0] + 2*PI;
        // if (mp[2][0] <= mp[1][0])
        // mp[2][0] = mp[2][0] + 2*PI;
        // RAn[0] = mp[0][0];
        // Dec[0] = mp[0][1];
        // Moonrise = false; // initialize
        // Moonset = false;
        //
        // for (k = 0; k < 24; k++) // check each hour of this day
        // {
        // ph = (k + 1)/24;
        //
        // RAn[2] = interpolate(mp[0][0], mp[1][0], mp[2][0], ph);
        // Dec[2] = interpolate(mp[0][1], mp[1][1], mp[2][1], ph);
        //
        // VHz[2] = test_moon(k, zone, t0, lat, mp[1][2]);
        // RAn[0] = RAn[2]; // advance to next hour
        // Dec[0] = Dec[2];
        // VHz[0] = VHz[2];
        // }
        // // display results
        // document.calc.moonrise.value = zintstr(Rise_time[0], 2) + "h" + zintstr(Rise_time[1], 2) + "m";
        // document.calc.moonset.value = zintstr(Set_time[0], 2) + "h" + zintstr(Set_time[1], 2) + "m";
        // document.calc.moonriseAz.value = azimut = frealstr(Rise_az, 5, 1) + "°";
        // document.calc.moonsetAz.value = azimut = frealstr(Set_az, 5, 1) + "°";
        // special_message();
        // }
        //
        // // Local Sidereal Time for zone
        // public void lst( lon, jd, z )
        // {
        // var s = 24110.5 + 8640184.812999999*jd/36525 + 86636.6*z + 86400*lon;
        // s = s/86400;
        // s = s - Math.floor(s);
        // return s*360*DR;
        // }
        //
        // // 3-point interpolation
        // public void interpolate( f0, f1, f2, p )
        // {
        // var a = f1 - f0;
        // var b = f2 - f1 - a;
        // var f = f0 + p*(2*a + b*(2*p - 1));
        // return f;
        // }
        //
        // // test an hour for an event
        // public void test_moon( k, zone, t0, lat, plx )
        // {
        // var ha = [0.0, 0.0, 0.0];
        // var a, b, c, d, e, s, z;
        // var hr, min, time;
        // var az, hz, nz, dz;
        // if (RAn[2] < RAn[0])
        // RAn[2] = RAn[2] + 2*PI;
        //
        // ha[0] = t0 - RAn[0] + k*K1;
        // ha[2] = t0 - RAn[2] + k*K1 + K1;
        //
        // ha[1] = (ha[2] + ha[0])/2; // hour angle at half hour
        // Dec[1] = (Dec[2] + Dec[0])/2; // declination at half hour
        // s = Math.sin(DR*lat);
        // c = Math.cos(DR*lat);
        // // refraction + sun semidiameter at horizon + parallax correction
        // z = Math.cos(DR*(90.567 - 41.685/plx));
        // if (k <= 0) // first call of public void
        // VHz[0] = s*Math.sin(Dec[0]) + c*Math.cos(Dec[0])*Math.cos(ha[0]) - z;
        // VHz[2] = s*Math.sin(Dec[2]) + c*Math.cos(Dec[2])*Math.cos(ha[2]) - z;
        //
        // if (sgn(VHz[0]) == sgn(VHz[2]))
        // return VHz[2]; // no event this hour
        //
        // VHz[1] = s*Math.sin(Dec[1]) + c*Math.cos(Dec[1])*Math.cos(ha[1]) - z;
        // a = 2*VHz[2] - 4*VHz[1] + 2*VHz[0];
        // b = 4*VHz[1] - 3*VHz[0] - VHz[2];
        // d = b*b - 4*a*VHz[0];
        // if (d < 0)
        // return VHz[2]; // no event this hour
        //
        // d = Math.sqrt(d);
        // e = (-b + d)/(2*a);
        // if (( e > 1 )||( e < 0 ))
        // e = (-b - d)/(2*a);
        // time = k + e + 1/120; // time of an event + round up
        // hr = Math.floor(time);
        // min = Math.floor((time - hr)*60);
        // hz = ha[0] + e*(ha[2] - ha[0]); // azimuth of the moon at the event
        // nz = -Math.cos(Dec[1])*Math.sin(hz);
        // dz = c*Math.sin(Dec[1]) - s*Math.cos(Dec[1])*Math.cos(hz);
        // az = Math.atan2(nz, dz)/DR;
        // if (az < 0) az = az + 360;
        //
        // if ((VHz[0] < 0)&&(VHz[2] > 0))
        // {
        // Rise_time[0] = hr;
        // Rise_time[1] = min;
        // Rise_az = az;
        // Moonrise = true;
        // }
        //
        // if ((VHz[0] > 0)&&(VHz[2] < 0))
        // {
        // Set_time[0] = hr;
        // Set_time[1] = min;
        // Set_az = az;
        // Moonset = true;
        // }
        // return VHz[2];
        // }
        //
        // // check for no moonrise and/or no moonset
        // public void special_message()
        // {
        // if((!Moonrise) && (!Moonset)) // neither moonrise nor moonset
        // {
        // if(VHz[2] < 0)
        // {
        // document.calc.moonmsg.value = "La Lune est sous l'horizon toute la journée";
        // document.calc.moonrise.value = "---";
        // document.calc.moonset.value = "---";
        // document.calc.moonriseAz.value = "---";
        // document.calc.moonsetAz.value = "---";
        // }
        // else
        // {
        // document.calc.moonmsg.value = "La Lune est visible toute la journée";
        // document.calc.moonrise.value = "---";
        // document.calc.moonset.value = "---";
        // document.calc.moonriseAz.value = "---";
        // document.calc.moonsetAz.value = "---";
        // }
        // }
        // else // moonrise or moonset
        // {
        // if(!Moonrise)
        // {
        // document.calc.moonmsg.value = "Pas de lever de Lune à cette date";
        // document.calc.moonrise.value = "---";
        // document.calc.moonriseAz.value = "---";
        // }
        // else if(!Moonset)
        // {
        // document.calc.moonmsg.value = "Pas de coucher de Lune à cette date";
        // document.calc.moonset.value = "---";
        // document.calc.moonsetAz.value = "---";
        // }
        // }
        // }
        //
        // // moon's position using fundamental arguments
        // // (Van Flandern & Pulkkinen, 1979)
        // public void moon( jd )
        // {
        // var d, f, g, h, m, n, s, u, v, w;
        // h = 0.606434 + 0.03660110129*jd;
        // m = 0.374897 + 0.03629164709*jd;
        // f = 0.259091 + 0.0367481952 *jd;
        // d = 0.827362 + 0.03386319198*jd;
        // n = 0.347343 - 0.00014709391*jd;
        // g = 0.993126 + 0.0027377785 *jd;
        // h = h - Math.floor(h);
        // m = m - Math.floor(m);
        // f = f - Math.floor(f);
        // d = d - Math.floor(d);
        // n = n - Math.floor(n);
        // g = g - Math.floor(g);
        // h = h*2*PI;
        // m = m*2*PI;
        // f = f*2*PI;
        // d = d*2*PI;
        // n = n*2*PI;
        // g = g*2*PI;
        // v = 0.39558*Math.sin(f + n);
        // v = v + 0.082 *Math.sin(f);
        // v = v + 0.03257*Math.sin(m - f - n);
        // v = v + 0.01092*Math.sin(m + f + n);
        // v = v + 0.00666*Math.sin(m - f);
        // v = v - 0.00644*Math.sin(m + f - 2*d + n);
        // v = v - 0.00331*Math.sin(f - 2*d + n);
        // v = v - 0.00304*Math.sin(f - 2*d);
        // v = v - 0.0024 *Math.sin(m - f - 2*d - n);
        // v = v + 0.00226*Math.sin(m + f);
        // v = v - 0.00108*Math.sin(m + f - 2*d);
        // v = v - 0.00079*Math.sin(f - n);
        // v = v + 0.00078*Math.sin(f + 2*d + n);
        //
        // u = 1 - 0.10828*Math.cos(m);
        // u = u - 0.0188 *Math.cos(m - 2*d);
        // u = u - 0.01479*Math.cos(2*d);
        // u = u + 0.00181*Math.cos(2*m - 2*d);
        // u = u - 0.00147*Math.cos(2*m);
        // u = u - 0.00105*Math.cos(2*d - g);
        // u = u - 0.00075*Math.cos(m - 2*d + g);
        //
        // w = 0.10478*Math.sin(m);
        // w = w - 0.04105*Math.sin(2*f + 2*n);
        // w = w - 0.0213 *Math.sin(m - 2*d);
        // w = w - 0.01779*Math.sin(2*f + n);
        // w = w + 0.01774*Math.sin(n);
        // w = w + 0.00987*Math.sin(2*d);
        // w = w - 0.00338*Math.sin(m - 2*f - 2*n);
        // w = w - 0.00309*Math.sin(g);
        // w = w - 0.0019 *Math.sin(2*f);
        // w = w - 0.00144*Math.sin(m + n);
        // w = w - 0.00144*Math.sin(m - 2*f - n);
        // w = w - 0.00113*Math.sin(m + 2*f + 2*n);
        // w = w - 0.00094*Math.sin(m - 2*d + g);
        // w = w - 0.00092*Math.sin(2*m - 2*d);
        // s = w/Math.sqrt(u - v*v); // compute moon's right ascension ...
        // Sky[0] = h + Math.atan(s/Math.sqrt(1 - s*s));
        // s = v/Math.sqrt(u); // declination ...
        // Sky[1] = Math.atan(s/Math.sqrt(1 - s*s));
        // Sky[2] = 60.40974*Math.sqrt( u ); // and parallax
        // }
        //
        // public void julian_day()
        // {
        // var a, b, jd;
        // var gregorian;
        // day = parseInt(document.calc.day.value);
        // month = parseInt(document.calc.month.value);
        // year = parseInt(document.calc.year.value);
        //
        // gregorian = (year < 1583) ? false : true;
        //
        // if ((month == 1)||(month == 2))
        // {
        // year = year - 1;
        // month = month + 12;
        // }
        // a = Math.floor(year/100);
        // if (gregorian) b = 2 - a + Math.floor(a/4);
        // else b = 0.0;
        // jd = Math.floor(365.25*(year + 4716))
        // + Math.floor(30.6001*(month + 1))
        // + day + b - 1524.5;
        //
        // return jd;
        // }
        //
        // // returns value for sign of argument
        // public void sgn( x )
        // {
        // var rv;
        // if (x > 0.0) rv = 1;
        // else if (x < 0.0) rv = -1;
        // else rv = 0;
        // return rv;
        // }
        //
        // // format a positive integer with leading zeroes
        // public void zintstr( num, width )
        // {
        // var str = num.toString(10);
        // var len = str.length;
        // var intgr = "";
        // var i;
        // for (i = 0; i < width - len; i++) // append leading zeroes
        // intgr += '0';
        // for (i = 0; i < len; i++) // append digits
        // intgr += str.charAt(i);
        // return intgr;
        // }
        //
        // // format an integer
        // public void cintstr( num, width )
        // {
        // var str = num.toString(10);
        // var len = str.length;
        // var intgr = "";
        // var i;
        // for (i = 0; i < width - len; i++) // append leading spaces
        // intgr += ' ';
        // for (i = 0; i < len; i++) // append digits
        // intgr += str.charAt(i);
        // return intgr;
        // }
        //
        // // format a real number
        // public void frealstr( num, width, fract )
        // {
        // var str = num.toFixed(fract);
        // var len = str.length;
        // var real = "";
        // var i;
        // for (i = 0; i < width - len; i++) // append leading spaces
        // real += ' ';
        // for (i = 0; i < len; i++) // append digits
        // real += str.charAt(i);
        // return real;
    }

}
