/**
 *  Ixion
 */
package fr.ixion.pi.astres.planetes;

import static fr.ixion.pi.Maths.*;
import static fr.ixion.pi.Soleil.*;

/**
 * @author Ixion
 */
public class Pluton extends Planete {

	/*
	 * (non-Javadoc)
	 * 
	 * @see fr.ixion.pi.planetes.Planete#constantes()
	 */
	@Override
	public void constantes() {
		const17 = 3.200;
	}

	@Override
    public void update()
    {
      //--------------- Longitude moyenne (l), demi-grand axe (a), excentricit� (e), inclinaison (i), 
      //--------longitude noeud ascendant (m)

    double  l = mod2pi((238.92881 + (522747.90 * T_2000 / 3600)) / r2d);  //longitude moyenne
      l = l - y * floor(l / y);
      double   a = 39.48168677 - (0.00076912 * T_2000);                      // demi-grand axe
      double    e = 0.24880766 + (0.00006465 * T_2000);                       // excentricit�
      double    i = (17.14175 + (11.07 * T_2000 / 3600)) / r2d;                 // inclinaison
      double    ap = (224.06676 - (132.25 * T_2000 / 3600)) / r2d;            // argument du p�rih�lie
      double    Om = (110.30347 - (37.33 * T_2000 / 3600)) / r2d;             // longitude du noeud ascendant

      double    m = (l - ap)      ;                                         // longitude du p�rih�lie

      m = m - y * floor(m / y);

      //--------�quation de Kepler

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

      double   r = a * (1 - e * cos(grand_e));

      //--------argument de latitude

      double    u = l + v - m - Om;
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

      //--------longitude �cliptique

      l = d + Om;
      if (l > 2 * PI) 
      {
      l = l - 2 * PI;
      }
      double  b = asin(sin(u) * sin(i));
      double  numerateur = r * cos(b) * sin(l - longitude_terre + PI);
      double  denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;
      l = atan(numerateur / denominateur) + longitude_terre + PI;
      if (l > 2 * PI)  
      {l = l - 2 * PI;}
      if (denominateur < 0) {l = l + PI;}

      double  diametre = const17;

      //--------conversion rectangulaire/polaire

      double    xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
      double     yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
      double     zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);

      double    distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));

   
      //--------�longation

      omega = 259.18 / r2d - 1934.142 / r2d * T_2000;
      l = l - 0.00479 / r2d * sin(omega);
      double    elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
      String elongation_planete;
	if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI)) 
      {elongation_planete = "Ouest";}
      else {elongation_planete = "Est";}

    
	double   elongationPluton = elongation;
   String   elongation_planetePluton =elongation_planete;

      String commentPluton;
	String quandPluton;
	if (elongationPluton < 20) {commentPluton = "Inobservable"; quandPluton = " ";}

      if (elongationPluton > 20 && elongationPluton < 45){
        commentPluton = "Observable aux instruments";
        if (elongation_planetePluton == "Ouest") {quandPluton = "en toute fin de nuit";}
        if (elongation_planetePluton == "Est") {quandPluton = "en tout d�but de soir�e";}}

      if (elongationPluton > 45 && elongationPluton < 120){
        commentPluton = "Observable aux instruments";
        if (elongation_planetePluton == "Ouest") {quandPluton = "en seconde partie de nuit";}
        if (elongation_planetePluton == "Est") {quandPluton = "en premi�re partie de nuit";}}

      if (elongationPluton > 120 && elongationPluton < 140) {commentPluton = "Observable aux instruments"; quandPluton = "pratiquement toute la nuit";}
      if (elongationPluton > 140 && elongationPluton < 180) {commentPluton = "Observable aux instruments"; quandPluton = "toute la nuit";}


    
      //--------convertion longitude et latitude en ascension droite et d�clinaison

      l = l -y * floor(l /y);
      double   l1 = l * r2d;
      d1 =((l1 - floor(l1)) * 60);
      d2 =((d1 - floor(d1)) * 60);

      double      beta = asin(r * sin(b) / distance_terre);
      double b1 = abs(beta * r2d);
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

      double ADPluton = asc * 15 / r2d;

      d = abs(declin);
      d1 = ((d - floor(d)) * 60);
      d2=  ((d1 - floor(d1)) * 60);

      double DecPluton;
	if (declin < 0) {signe = "-"; DecPluton = - d / r2d;}
      else {signe = "+"; DecPluton = + d / r2d;}

    double  ascPluton = asc;
    		double  declinPluton = declin;

      //Magnitude de la plan�te

    		double   dist = R ;                   // rayon vecteur Soleil-Terre
    				double    ray = r         ;            // rayon vecteur Soleil-plan�te
    						double    delta = distance_terre  ;    // distance Terre-plan�te

    						double   FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) );
          FV = (FV) * r2d;

        		  double  phase = (1 + cos(FV / r2d)) * 50;
         
        		  double lgrd = log(ray * delta) / log(10);
    						double   magnitude = - 0.14 + 5 * lgrd;
    }

	@Override
	public int getID() {
		// TODO Auto-generated method stub
		return 7;
	}

}