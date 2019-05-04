/**
 *  Ixion
 */
package fr.ixion.pi;

import static fr.ixion.pi.Maths.*;

/**
 * @author Ixion
 */
public class Soleil
{

    public static double latdeg;
    public static int latmin;
    public static double londeg;
    public static int lonmin;
    public static double T;
    public static double T_2000;
    public static double T2;
    public static double Lo;
    public static double T3;
    public static double M;
    public static double anomalie_soleil;
    public static double e;
    public static double c;
    public static double Lov;
    public static double v;
    public static double omega;
    public static double R;
    public static double longapp;
    public static double longapp0;
    public static double longapp1;
    public static double d1;
    public static double d2;
    public static double A;
    public static double B;
    public static double C;
    public static double D;
    public static double E;
    public static double H;
    public static double longitude;
    public static double longitude1;
    public static double long1;
    public static double obliquite;
    public static double obliquite1;
    public static double eps;
    public static double yT;
    public static double eqT;
    public static double asc;
    public static double ADSoleil;
    public static double declin;
    public static double d;
    public static double DecSoleil;
    public static double ascSoleil;
    public static double declinSoleil;
    public static double longitude_soleil;
    public static double longitude_terre;
    public static int latitude_terre;
    public static double distance_terre_soleil;
    public static double longitude_vraie;
    public static double xs;
    public static double ys;
    public static double zs;
	public static double T2_2000;
	public static double T3_2000;
	public static double T4_2000;

    public Soleil()
    {
        update();
    }

    /**
     * 
     */
    public static void update()
    {
    	
    	Temps.instance.updateDate();
        // SOLEIL

        // --------------- Données pour circonstances locales

        latdeg = Location.instance.latitude;
        latmin = 0;
        londeg = Location.instance.longitude;
        lonmin = 0;

        T = Temps.instance.T;

        T2 = Temps.instance.T2;

        T3 = Temps.instance.T3;

        // --------------- Longitude moyenne du Soleil, rapportée à l'équinoxe moyen de la date considérée

        Lo = 279.69668 + 36000.76892 * T + 0.0003025 * T2;
        Lo = Lo / r2d;

        // --------------- Anomalie moyenne du Soleil

        M = (358.47583 + 35999.04975 * T) - 0.00015 * T2 - 0.0000033 * T3;
        M = M / r2d;
        anomalie_soleil = M;

        // --------------- Excentricité de l'orbite terrestre

        e = 0.01675104 - 0.0000418 * T - 0.000000126 * T2;

        // --------------- Equation de centre C du Soleil

        c = (1.919460 - 0.004789 * T - 0.000014 * T2) * sin(M) + (0.020094 - 0.000100 * T) * sin(2 * M) + 0.000293 * sin(3 * M);
        c = c / r2d;

        // --------------- Longitude vraie du Soleil rapportée à l'équinoxe moyen de la date considéré, Anomalie vraie

        Lov = Lo + c;
        v = M + c;

        // --------------- Rayon vecteur

        R = 1.0000002 * (1 - e * e) / (1 + e * cos(v));

        // --------------- Longitude apparente du Soleil rapportée à l'équinoxe vrai de l'époque

        omega = 259.18 / r2d - 1934.142 / r2d * T;
        longapp = Lov - 0.00569 / r2d - 0.00479 / r2d * sin(omega);

        longapp0 = longapp - y * floor(longapp / y);
        longapp1 = longapp0 * r2d;

        d1 = ((longapp1 - floor(longapp1)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        // ---------------Corrections à appliquer à la longitude solaire et au rayon vecteur pour une meilleur précision :
        // ------- A et B sont les corrections dues à l'action de Vénus, C à celle de Jupiter,
        // ------- D à celle de la Lune, alors que E est une inégalité de longue période

        A = 153.23 / r2d + 22518.7541 / r2d * T;
        B = 216.57 / r2d + 45037.5082 / r2d * T;
        C = 312.69 / r2d + 32964.3577 / r2d * T;
        D = 350.74 / r2d + 445267.1142 / r2d * T - 0.00144 / r2d * T2;
        E = 231.19 / r2d + 20.20 / r2d * T;
        H = 353.40 / r2d + 65928.7155 / r2d * T;

        longitude = longapp + 0.00134 / r2d * cos(A) + 0.00154 / r2d * cos(B) + 0.002 / r2d * cos(C) + 0.00179 / r2d * sin(D) + 0.00178 / r2d * sin(E);
        longitude1 = longapp - y * floor(longapp / y);
        long1 = longitude1 * r2d;

        d1 = ((long1 - floor(long1)) * 60);
        d2 = ((d1 - floor(d1)) * 60);

        R = R + 0.00000543 * sin(A) + 0.00001575 * sin(B) + 0.00001627 * sin(C) + 0.00003076 * cos(D) + 0.00000927 * sin(H);

        // --------------- Obliquité de l'écliptique

        obliquite = (23.452294 - 0.0130125 * T - 0.00000164 * T2 + 0.000000503 * T3 + 0.00256 * cos(omega)) / r2d;
        obliquite1 = (23.452294 - 0.0130125 * T - 0.00000164 * T2 + 0.000000503 * T3) / r2d;

        // --------------- Equation du Temps (formule de W.M. SMART - "Text-Book on Spherical Astronomy" page 19 - édition de 1956)

        eps = obliquite;

        yT = tan(eps / 2) * tan(eps / 2);

        eqT = -yT * sin(2 * Lo) + 2 * e * sin(M) - 4 * e * yT * sin(M) * cos(2 * Lo) + 0.5 * yT * yT * sin(4 * Lo) + 5 / 4 * e * e * sin(2 * M);
        eqT = eqT * r2d;
        // --------------- Coordonnées du Soleil : Ascension droite

        asc = r2d / 15 * atan(cos(obliquite) * sin(longitude) / cos(longitude));

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

        ADSoleil = asc * 15 / r2d;

        // --------------- Coordonnées du Soleil : Déclinaison

        declin = r2d * asin(sin(obliquite) * sin(longitude));

        d = abs(declin);
        d1 = ((d - floor(d)) * 60);
        d2 = ((d1 - floor(d1)) * 60);
        DecSoleil = 0;
        if(declin < 0)
        {
            DecSoleil = -d / r2d;
        }
        else
        {
            DecSoleil = +d / r2d;
        }

        ascSoleil = asc;
        declinSoleil = declin;

        // --------------------------------------------- Calcul de la position des planètes

        // longitude_soleil = Lov - y * floor(Lov / y);
        longitude_soleil = longitude - y * floor(longitude / y);
        longitude_terre = longitude_soleil + PI;

        latitude_terre = 0;

        distance_terre_soleil = R;

        longitude_vraie = longitude_soleil;

        // --------------- Coordonnées rectangulaires équatoriales du Soleil

        xs = R * cos(longitude_vraie);
        ys = R * sin(longitude_vraie) * cos(obliquite);
        zs = R * sin(longitude_vraie) * sin(obliquite);
        
        T_2000 = Temps.instance.T_2000;
        T2_2000 = T_2000 * T_2000;
        T3_2000 = T2_2000 * T_2000;
        T4_2000 = T2_2000 * T2_2000;
    }
}
