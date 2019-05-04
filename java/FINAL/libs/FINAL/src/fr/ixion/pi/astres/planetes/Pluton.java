/**
 *  Ixion
 */
package fr.ixion.pi.astres.planetes;

import static fr.ixion.pi.Maths.*;
import static fr.ixion.pi.Soleil.*;
/**
 * @author Ixion
 */
public class Pluton extends Planete
{

    /*
     * (non-Javadoc)
     * @see fr.ixion.pi.planetes.Planete#constantes()
     */
    @Override
    public void constantes()
    {
        const17 = 3.200;
    }

    @Override
    public void update()
    {
      //--------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i), 
      //--------longitude noeud ascendant (m)

    double  l = mod2pi((238.92881 + (522747.90 * T_2000 / 3600)) / r2d);  //longitude moyenne
      l = l - y * floor(l / y);
      double   a = 39.48168677 - (0.00076912 * T_2000);                      // demi-grand axe
      double    e = 0.24880766 + (0.00006465 * T_2000);                       // excentricité
      double    i = (17.14175 + (11.07 * T_2000 / 3600)) / r2d;                 // inclinaison
      double    ap = (224.06676 - (132.25 * T_2000 / 3600)) / r2d;            // argument du périhélie
      double    Om = (110.30347 - (37.33 * T_2000 / 3600)) / r2d;             // longitude du noeud ascendant

      double    m = (l - ap)      ;                                         // longitude du périhélie

      m = m - y * floor(m / y);

      //--------équation de Kepler

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

      //--------anomalie vraie

      v = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(grand_e / 2));
      if (v < 0) {v = v + 2 * PI;}

      //--------rayon vecteur

      r = a * (1 - e * cos(grand_e));

      //--------argument de latitude

      u = l + v - m - Om;
      u = u - y * floor(u / y);
      if (cos(u) != 0) {
         d = atan(cos(i) * tan(u));
         if (cos(u) < 0) {
            d = d + PI;
         }
      }
         else {
           d = u;
         }

      //--------longitude écliptique

      l = d + Om;
      if (l > 2 * PI) 
      {
      l = l - 2 * PI;
      }
      b = asin(sin(u) * sin(i));
      numerateur = r * cos(b) * sin(l - longitude_terre + PI);
      denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;
      l = atan(numerateur / denominateur) + longitude_terre + PI;
      if (l > 2 * PI)  
      {l = l - 2 * PI;}
      if (denominateur < 0) {l = l + PI;}

      diametre = const17;

      //--------conversion rectangulaire/polaire

      xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
      yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
      zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);

      distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));

      document.calc.diametrepluton.value = floor(diametre / distance_terre * 100 + 0.5) / 100 + "''";

      //--------élongation

      omega = 259.18 / r2d - 1934.142 / r2d * T_2000;
      l = l - 0.00479 / r2d * sin(omega);
      elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
      if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI)) 
      {elongation_planete = "Ouest";}
      else {elongation_planete = "Est";}

      document.calc.elongatpluton.value = floor(elongation * 10 + 0.5) / 10 + "° " + elongation_planete;

      elongationPluton = elongation
      elongation_planetePluton =elongation_planete

      if (elongationPluton < 20) {commentPluton = "Inobservable"; quandPluton = " "}

      if (elongationPluton > 20 && elongationPluton < 45){
        commentPluton = "Observable aux instruments"
        if (elongation_planetePluton == "Ouest") {quandPluton = "en toute fin de nuit"}
        if (elongation_planetePluton == "Est") {quandPluton = "en tout début de soirée"}}

      if (elongationPluton > 45 && elongationPluton < 120){
        commentPluton = "Observable aux instruments"
        if (elongation_planetePluton == "Ouest") {quandPluton = "en seconde partie de nuit"}
        if (elongation_planetePluton == "Est") {quandPluton = "en première partie de nuit"}}

      if (elongationPluton > 120 && elongationPluton < 140) {commentPluton = "Observable aux instruments"; quandPluton = "pratiquement toute la nuit"}
      if (elongationPluton > 140 && elongationPluton < 180) {commentPluton = "Observable aux instruments"; quandPluton = "toute la nuit"}


      document.calc.visibpluton.value = commentPluton + " " + quandPluton

      //--------convertion longitude et latitude en ascension droite et déclinaison

      l = l -y * floor(l /y);
      l1 = l * r2d;
      d1 =((l1 - floor(l1)) * 60);
      d2 =((d1 - floor(d1)) * 60);

      document.calc.longpluton.value = floor(l1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";

      beta = asin(r * sin(b) / distance_terre);
      b1 = abs(beta * r2d);
      d1 = ((b1 - floor(b1)) * 60);
      d2 = ((d1 - floor(d1)) * 60);
      if (beta < 0) {signe = "-";}
      else {signe = "+";}

      document.calc.latpluton.value = signe + floor(b1) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
      document.calc.rayonpluton.value = floor(r * 100000 + 0.5) / 100000 + " UA"
      document.calc.distancepluton.value = floor(distance_terre * 100000 + 0.5) / 100000 + " UA"

      asc = atan((cos(obliquite) * sin(l) - tan(beta) * sin(obliquite)) / cos(l));
      if (asc < 0) {asc = asc + 2 * PI;}
      if (cos(l) < 0) {asc = asc + PI;}
      if (asc > 2 * PI) {asc = asc - 2 * PI;}
      declin = r2d * asin(sin(beta) * cos(obliquite) + cos(beta) * sin(obliquite) * sin(l));
      asc = asc * r2d / 15;
      d1 = ((asc - floor(asc)) * 60);
      d2 = ((d1 - floor(d1)) * 60);

      document.calc.alphapluton.value = floor(asc) + "h" + zero(floor(d1)) + "m" + zero(floor(d2 * 100 + 0.5) / 100) + "s";
      ADPluton = asc * 15 / r2d

      d = abs(declin);
      d1 = ((d - floor(d)) * 60);
      d2=  ((d1 - floor(d1)) * 60);

      if (declin < 0) {signe = "-"; DecPluton = - d / r2d}
      else {signe = "+"; DecPluton = + d / r2d}

      document.calc.deltapluton.value = signe + floor(d) + "°" + zero(floor(d1)) + "'" + zero(floor(d2 * 100 + 0.5) / 100) + "''";
      ascPluton = asc
      declinPluton = declin

      //Magnitude de la planète

      dist = R                    // rayon vecteur Soleil-Terre
      ray = r                     // rayon vecteur Soleil-planète
      delta = distance_terre      // distance Terre-planète

          FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) )
          FV = (FV) * r2d

          phase = (1 + cos(FV / r2d)) * 50
         
          lgrd = log(ray * delta) / log(10);
          magnitude = - 0.14 + 5 * lgrd;
    }
    @Override
    public int getID()
    {
        // TODO Auto-generated method stub
        return 7;
    }

}
