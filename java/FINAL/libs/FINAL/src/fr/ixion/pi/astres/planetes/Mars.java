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
public class Mars extends Planete
{

    /* (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#constantes()
     */
    @Override
    public void constantes()
    {
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
    }
    @Override
    public int getID()
    {
        // TODO Auto-generated method stub
        return 1;
    }
    /* (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#update()
     */
    @Override
    public void update()
    {
     //--------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i), 
     //--------longitude noeud ascendant (m)

  double   l = (const1 + const2 * T + const3 * T2) / r2d;
     l = l - y * floor(l / y);

     double  a = const4;

     double  e = const5 + const6 * T + const7 * T2;
     double  i = (const8 + const9 * T + const10 * T2) / r2d;
     double  m = (const11 + const12 * T + const13 * T2) / r2d;
     m = m - y * floor(m / y);
     double   longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;

     //--------------- Anomalie moyenne

     double   m1 = (102.27938 + 149472.51529 * T + 0.000007 * T2) / r2d;
     double   m2 = (212.60322 + 58517.80387 * T + 0.001286 * T2) / r2d;
     double   m4 = (319.51913 + 19139.85475 * T + 0.000181 * T2) / r2d;
     double   m5 = (225.32833 + 3034.69202 * T - 0.000722 * T2) / r2d;
     double   m6 = (175.46622 + 1221.55147 * T - 0.000502 * T2) / r2d;

     m1= m1 - y * floor(m1 / y);
     m2= m2 - y * floor(m2 / y);
     m4 = m4 - y * floor(m4 / y);
     m5 = m5 - y * floor(m5 / y);
     m6 = m6 - y * floor(m6 / y);

     //--------corrections

     double     corr = -0.01133 / r2d * sin(3 * m5 - 8 * m4 + 4 * m) - 0.00933 / r2d * cos(3 * m5 - 8 * m4 + 4 * m);
        l = l + corr;
        m = m + corr;

     //--------équation de Kepler

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


     //--------anomalie vraie

     v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(grand_e / 2));
     if (v < 0) {v = v + 2 * PI;}

     //--------rayon vecteur

     double  r = a * (1 - e * cos(grand_e));

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

     //--------argument de latitude

        double    u = l + v - m - longitude_noeud;
     u = u - y * floor(u / y);

     if (cos(u) != 0) {d = atan(cos(i) * tan(u));
        if (cos(u) < 0) {d = d + PI;}
     }
        else {d = u;}

     //--------longitude écliptique

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

     if (l > 2 * PI) {l = l - 2 * PI;}

     double  b = asin(sin(u) * sin(i));


     double  numerateur = r * cos(b) * sin(l - longitude_terre + PI);
     double   denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;

     l = atan(numerateur / denominateur) + longitude_terre + PI;

     if (l > 2 * PI) {l = l - 2 * PI;}

     if (denominateur < 0) {l = l + PI;}

     double diametre = const17;

     //--------conversion rectangulaire/polaire

     double   xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
     double  yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
     double     zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);

     double  distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));

     //--------élongation

     omega = 259.18 / r2d - 1934.142 / r2d * T;
     l = l - 0.00479 / r2d * sin(omega);

     double  elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
     String elongation_planete;
    if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI)) 
     {elongation_planete = "Ouest";}
     else {elongation_planete = "Est";}

     
    double   elongationMars = elongation;
    String  elongation_planeteMars = elongation_planete;

     String commentMars;
    String quandMars;
    if (elongationMars < 10) {commentMars = "Inobservable"; quandMars = " ";}

     if (elongationMars > 10 && elongationMars < 20){
       commentMars = "Difficilement observable";
       if (elongation_planeteMars == "Ouest") {quandMars = "peu avant le lever du Soleil";}
       if (elongation_planeteMars == "Est") {quandMars = "au coucher du Soleil";}}

     if (elongationMars > 20 && elongationMars < 45){
       commentMars = "Observable";
       if (elongation_planeteMars == "Ouest") {quandMars = "en toute fin de nuit";}
       if (elongation_planeteMars == "Est") {quandMars = "en tout début de soirée";}}

     if (elongationMars > 45 && elongationMars < 120){
       commentMars = "Observable";
       if (elongation_planeteMars == "Ouest") {quandMars = "en seconde partie de nuit";}
       if (elongation_planeteMars == "Est") {quandMars = "en première partie de nuit";}}

     if (elongationMars > 120 && elongationMars < 140) {commentMars = "Observable"; quandMars = "pratiquement toute la nuit";}
     if (elongationMars > 140 && elongationMars < 180) {commentMars = "Observable"; quandMars = "toute la nuit";}


     //--------convertion longitude et latitude en ascension droite et déclinaison

     l = l -y * floor(l /y);
     double  l1 = l * r2d;

     d1 = ((l1 - floor(l1)) * 60);
     d2 = ((d1 - floor(d1)) * 60);

   
     double  beta =asin(r * sin(b) / distance_terre);

     double   b1 = abs(beta * r2d);
     d1 = ((b1 - floor(b1)) * 60);
     d2 = ((d1 - floor(d1)) * 60);

     String signe;
    if (beta < 0) {signe = "-";}
     else {signe = "+";}

       asc = atan((cos(obliquite) * sin(l) - tan(beta) * sin(obliquite)) / cos(l));
     if (asc < 0) {asc = asc + 2 * PI;}
     if (cos(l) < 0) {asc = asc + PI;}
     if (asc > 2 * PI) {asc = asc - 2 * PI;}
     declin = r2d * asin(sin(beta) * cos(obliquite) + cos(beta) * sin(obliquite) * sin(l));

     asc = asc * r2d / 15;

     d1 = ((asc - floor(asc)) * 60);
     d2 = ((d1 - floor(d1)) * 60);

     double  ADMars = asc * 15 / r2d;

     d = abs(declin);
     d1 = ((d - floor(d)) * 60);
     d2=  ((d1 - floor(d1)) * 60);

     double DecMars;
    if (declin < 0) {signe = "-"; DecMars = - d / r2d;}
     else {signe = "+"; DecMars = + d / r2d;}

     double ascMars = asc;
     double declinMars = declin;

     //Magnitude de la planète

     double  dist = R                 ;   // rayon vecteur Soleil-Terre
     double ray = r                 ;    // rayon vecteur Soleil-planète
     double  delta = distance_terre ;     // distance Terre-planète

     double   FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) );
         FV = (FV) * r2d;

         double   phase = (1 + cos(FV / r2d)) * 50;;

         double   magnitude = - 1.51 + 5 * (log(ray * delta))/ log(10) + 0.016 * FV;
     magnitude = ceil(magnitude * 10) / 10;
  }

}
