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
public class Venus extends Planete
{

    /* (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#constantes()
     */
    @Override
    public void constantes()
    {
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

    }
    @Override
    public int getID()
    {
        // TODO Auto-generated method stub
        return 3;
    }
    /* (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#update()
     */
    @Override
    public void update()
    {
      //--------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i), 
      //--------longitude noeud ascendant (m)

      double l = (const1 + const2 * T + const3 * T2) / r2d;
      l = l - y * floor(l / y);

      double a = const4;

      e = const5 + const6 * T + const7 * T2;
      double i = (const8 + const9 * T + const10 * T2) / r2d;
      double m = (const11 + const12 * T + const13 * T2) / r2d;
      m = m - y * floor(m / y);
      double  longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;

      //--------------- Anomalie moyenne

      double    m1 = (102.27938 + 149472.51529 * T + 0.000007 * T2) / r2d;
      double    m2 = (212.60322 + 58517.80387 * T + 0.001286 * T2) / r2d;
      double    m4 = (319.51913 + 19139.85475 * T + 0.000181 * T2) / r2d;
      double   m5 = (225.32833 + 3034.69202 * T - 0.000722 * T2) / r2d;
      double  m6 = (175.46622 + 1221.55147 * T - 0.000502 * T2) / r2d;

      m1= m1 - y * floor(m1 / y);
      m2= m2 - y * floor(m2 / y);
      m4 = m4 - y * floor(m4 / y);
      m5 = m5 - y * floor(m5 / y);
      m6 = m6 - y * floor(m6 / y);

         l = l + 0.00077 / r2d * sin(237.24 / r2d + 150.27 / r2d * T);
         m = m + 0.00077 / r2d * sin(237.24 / r2d + 150.27 / r2d * T);

      //--------équation de Kepler

         double  grand_e = m;
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

         r = r + 0.000022501 * cos(2 * m - 2 * m2 - 58.208 / r2d);
         r = r + 0.000019045 * cos(3 * m - 3 * m2 + 92.577 / r2d);
         r = r + 0.000006887 * cos(m5 - m2 - 118.09 / r2d);
         r = r + 0.000005172 * cos(m - m2 - 29.11 / r2d);
         r = r + 0.00000362 * cos(5 * m - 4 * m2 - 104.208 / r2d);
         r = r + 0.000003283 * cos(4 * m - 4 * m2 + 63.513 / r2d);
         r = r + 0.000003074 * cos(2 * m5 - 2 * m2 - 55.167 / r2d);

      //--------argument de latitude

         double    u = l + v - m - longitude_noeud;
      u = u - y * floor(u / y);

      if (cos(u) != 0) { d = atan(cos(i) * tan(u));
         if (cos(u) < 0) {d = d + PI;}
      }
         else {d = u;}

      //--------longitude écliptique

      l = d + longitude_noeud;

         l = l + 0.00313 / r2d * cos(2 * m - 2 * m2 - 148.225 / r2d);
         l = l + 0.00198 / r2d * cos(3 * m - 3 * m2 + 2.565 / r2d);
         l = l + 0.00136 / r2d * cos(m - m2 - 119.107 / r2d);
         l = l + 0.00096 / r2d * cos(3 * m - 2 * m2 - 135.912 / r2d);
         l = l + 0.00082 / r2d * cos(m5 - m2 - 208.087 / r2d);

      if (l > 2 * PI) {l = l - 2 * PI;}

      double   b = asin(sin(u) * sin(i));

      double   numerateur = r * cos(b) * sin(l - longitude_terre + PI);
      double   denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;

      l = atan(numerateur / denominateur) + longitude_terre + PI;

      if (l > 2 * PI) {l = l - 2 * PI;}

      if (denominateur < 0) {l = l + PI;}

      double    diametre = const17;

      //--------conversion rectangulaire/polaire

      double  xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
      double    yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
      double    zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);

      double     distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));

   
      //--------élongation

      omega = 259.18 / r2d - 1934.142 / r2d * T;
      l = l - 0.00479 / r2d * sin(omega);

      double    elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
      String elongation_planete;
    if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI))
      {elongation_planete = "Ouest";}
      else {elongation_planete = "Est";}

      
      double elongationVen = elongation;
      String elongation_planeteVen = elongation_planete;

      String commentVen;
    String quandVen;
    if (elongationVen < 10) {commentVen = "Inobservable"; quandVen = " ";}

      if (elongationVen > 10 && elongationVen < 20){
        commentVen = "Difficilement visible";
        if (elongation_planeteVen == "Ouest") {quandVen = "peu avant le lever du Soleil";}
        if (elongation_planeteVen == "Est") {quandVen = "au coucher du Soleil";}}

      if (elongationVen > 20 && elongationVen < 50){
        commentVen = "Visible";
        if (elongation_planeteVen == "Ouest") {quandVen = "en toute fin de nuit";}
        if (elongation_planeteVen == "Est") {quandVen = "en tout début de soirée";}}


      //--------convertion longitude et latitude en ascension droite et déclinaison

      l = l - y * floor(l / y);
      double l1 = l * r2d;

      d1 = ((l1 - floor(l1)) * 60);
      d2 = ((d1 - floor(d1)) * 60);

    
      double  beta = asin(r * sin(b) / distance_terre);

      double    b1 = abs(beta * r2d);
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
      d2=  ((d1 - floor(d1)) * 60);

      double      ADVen = asc * 15 / r2d;

      d = abs(declin);
      d1 = ((d - floor(d)) * 60);
      d2=  ((d1 - floor(d1)) * 60);

      double DecVen;
    if (declin < 0) {signe = "-"; DecVen = - d / r2d;}
      else {signe = "+"; DecVen = + d / r2d;}

    double ascVen = asc;
    double  declinVen = declin;

      //Magnitude de la planète

    double  dist = R    ;                // rayon vecteur Soleil-Terre
    double ray = r      ;               // rayon vecteur Soleil-planète
    double delta = distance_terre ;     // distance Terre-planète

    double      FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) );
          FV = (FV) * r2d;

          double        phase = (1 + cos(FV / r2d)) * 50;

          double magnitude = - 4.34 + 5 * (log(ray * delta))/ log(10) + 0.013 * FV + 4.2E-7 * pow(FV,3);
      magnitude = ceil(magnitude * 10) / 10;

     
    }

}
