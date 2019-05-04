package fr.ixion.pi.astres.satellites;

import static fr.ixion.pi.Maths.*;
import static fr.ixion.pi.Soleil.*;

import fr.ixion.pi.Utils;

public class Lune extends Satellite{

	@Override
	public int getID() {
		// TODO Auto-generated method stub
		return 8;
	}

	@Override
	public void update() {
		// PLUTON Nécessaire pour calculer
		double const17 = 3.200;
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

	      //--------longitude écliptique

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

	   
	      //--------élongation

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
	        if (elongation_planetePluton == "Est") {quandPluton = "en tout début de soirée";}}

	      if (elongationPluton > 45 && elongationPluton < 120){
	        commentPluton = "Observable aux instruments";
	        if (elongation_planetePluton == "Ouest") {quandPluton = "en seconde partie de nuit";}
	        if (elongation_planetePluton == "Est") {quandPluton = "en première partie de nuit";}}

	      if (elongationPluton > 120 && elongationPluton < 140) {commentPluton = "Observable aux instruments"; quandPluton = "pratiquement toute la nuit";}
	      if (elongationPluton > 140 && elongationPluton < 180) {commentPluton = "Observable aux instruments"; quandPluton = "toute la nuit";}


	    
	      //--------convertion longitude et latitude en ascension droite et déclinaison

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

	      //Magnitude de la planète

	    		double   dist = R ;                   // rayon vecteur Soleil-Terre
	    				double    ray = r         ;            // rayon vecteur Soleil-planète
	    						double    delta = distance_terre  ;    // distance Terre-planète

	    						double   FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) );
	          FV = (FV) * r2d;

	        		  double  phase = (1 + cos(FV / r2d)) * 50;
	         
	        		  double lgrd = log(ray * delta) / log(10);
	    						double   magnitude = - 0.14 + 5 * lgrd;

		
		double ct = 0.00000484814;
		double longitude_soleil = longitude + 0.00134 / r2d * cos(a) + 0.00154 / r2d * cos(b) + 0.002 / r2d * cos(c) + 0.00179 / r2d * sin(d) + 0.00178 / r2d * sin(e);
		longitude_soleil = longitude_soleil - y * floor(longitude_soleil / y);

			e = 1 - 0.002516 * T_2000 - 0.0000074 * T2_2000;
				double excentricite = e * 0.01675104;

		double	mercure = (252.250906 + 149472.674636 * T_2000) / r2d;
			mercure = mercure - y * floor(mercure /y);
		double	venus = (181.979801 + 58517.815676 * T_2000) / r2d;
			venus = venus - y * floor(venus / y);
		double	longitude_moyenne_terre = (100.46645 + 35999.372854 * T_2000) / r2d;
		longitude_moyenne_terre = longitude_moyenne_terre - y * floor(longitude_moyenne_terre /y);
		double mars = (355.433275 + 19140.299331 * T_2000) / r2d;
		mars = mars - y * floor(mars / y);
		double	jupiter = (34.351484 + 3034.905675 * T_2000) / r2d;
		jupiter = jupiter - y * floor(jupiter / y);
		double	saturne = (50.077471 + 1222.113794 * T_2000) / r2d;
		saturne = saturne - y * floor(saturne /y);
		double	uranus = ((314 + 3 / 60 + 18.01841 / 3600) + 1542481.19393 / 3600 * T_2000) / r2d;
		uranus = uranus - y * floor(uranus / y);
		double	neptune = ((304 + 20 / 60 + 55.19575 / 3600) + 786550.32074 / 3600 * T_2000) / r2d;
		neptune = neptune - y * floor(neptune / y);

		double a1=(119.75 + 131.849 * T_2000) / r2d;
		double a2=(53.09 + 479264.29 * T_2000) / r2d;
		double a3=(313.45 + 481266.484 * T_2000) / r2d;

		double	om = (125.044555 - 1934.1361849 * T_2000 + 0.0020762 * T2_2000 + T3_2000 / 467410 - T4_2000 / 60616000) / r2d;
		double longitude_noeud = om;

		l = (218.3164591 + 481267.88134236 * T_2000 - 0.0013268 * T2_2000 + T3_2000 / 538841 -  T4_2000 / 65194000) / r2d;
		l = l - y * floor(l / y);
		double	w1 = l;

			m = (357.5291092 + 35999.0502909 * T_2000 - 0.0001536 * T2_2000 + T3_2000 / 24490000) / r2d;
		double	l_prime = m;
		double	anomalie_lune = m;

		double	f = (93.2720993 + 483202.0175273 * T_2000 - 0.0034029 * T2_2000 -  T3_2000/3526000 + T4_2000 / 863310000) / r2d;
		f = f - y * floor(f / y);

		double	n = (134.9634114 + 477198.8676313 * T_2000 + 0.008997 * T2_2000 + T3_2000 / 69699 - T4_2000 / 14712000) /r2d;
		n = n - y * floor(n / y);
		double	petit_l = n;

		d = (297.8502042 + 445267.1115168 * T_2000 - 0.00163 * T2_2000 + T3_2000 / 545868 - T4_2000 / 113065000) / r2d;
		d = d - y * floor(d / y);

		double	dzeta_lune = w1 + 5029.0966 / 3600 / r2d * T_2000;

		double	nu = (-171996 - 174.2 * T_2000) * sin(om);
		nu = nu + (-13187 - 1.6 * T_2000) * sin(-2 * d + 2 * f + 2 * om);
		nu = nu + (-2274 - 0.2 * T_2000) * sin(2 * f + 2 * om);
		nu = nu + (2062 + 0.2 * T_2000) * sin(2 * om);
		nu = nu + (1426 - 3.4 * T_2000) * sin(m);
		nu = nu + (712 + 0.1 * T_2000) * sin(n);
		nu = nu + (-517 + 1.2 * T_2000) * sin(-2 * d + m + 2 * f + 2 * om);
		nu = nu + (-386 - 0.4 * T_2000) * sin(2 * f + om);
		nu = nu - 301 * sin(n + 2 * f + 2 * om);
		nu = nu + (217 - 0.5 * T_2000) * sin(-2 * d - m + 2 * f + 2 * om);
		nu = nu - 158 * sin(-2 * d + n);
		nu = nu + (129 + 0.1 * T_2000)  * sin (-2 * d + 2 * f + om);
		nu = nu + 123  * sin (-n + 2 * f + 2 * om);
		nu = nu + 63 * sin(2 * d);
		nu = nu + (63 + 0.1 * T_2000)  * sin (n + om);
		nu = nu - 59 * sin(2 * d - n + 2 * f + 2 * om);
		nu = nu - (58 - 0.1 * T_2000) * sin(-n + om);
		nu = nu - 51 * sin(n + 2 * f + om);
		nu = nu + 48 * sin(-2 * d + 2 * n);
		nu = nu + 46 * sin(-2 * n + 2 * f + om);
		nu = nu - 38 * sin(2 * (d + f + om));
		nu = nu - 31 * sin(2 * (n + f + om));
		nu = nu + 29 * sin(2 * n);
		nu = nu + 29 * sin(-2 * d + n + 2 * f + 2 * om);
		nu = nu + 26 * sin(2 * f);
		nu = nu - 22 * sin(-2 * d + 2 * f);
		nu = nu + 21 * sin(-n + 2 * f + om);
		nu = nu + (17 - 0.1 * T_2000) * sin(2 * m);
		nu = nu + 16 * sin(2 * d - n + om);
		nu = nu - (16 + 0.1 * T_2000) * sin(-2 * d + 2 * m + 2 * f + 2 * om);
		nu = nu - 15 * sin(m + om);
		nu = nu - 13 * sin(-2 * d + n + om);
		nu = nu - 12 * sin(-m - om);
		nu = nu + 11 * sin(2 * n - 2 * f);
		nu = nu - 10 * sin(2 * d - n + 2 * f + om);
		nu = nu - 8 * sin(2 * d + n + 2 * f + 2 * om);
		nu = nu + 7 * sin(m + 2 * f + 2 * om);
		nu = nu - 7 * sin(-2 * d + m + n);
		nu = nu - 7 * sin(-m + 2 * f + 2 * om);
		nu = nu - 7 * sin(2 * d + 2 * f + om);
		nu = nu + 6 * sin(2 * d + n);
		nu = nu + 6 * sin(-2 * d + 2 * n + 2 * f + 2 * om);
		nu = nu + 6 * sin(-2 * d + n + 2 * f + om);
		nu = nu - 6 * sin(2 * d - 2 * n + om);
		nu = nu - 6 * sin(2 * d + om);
		nu = nu + 5 * sin(-m + n);
		nu = nu - 5 * sin(-2 * d - m + 2 * f + om);
		nu = nu - 5 * sin(-2 * d + om);
		nu = nu - 5 * sin(2 * n + 2 * f + om);
		nu = nu + 4 * sin(-2 * d + 2 * n + om);
		nu = nu + 4 * sin(-2 * d + m + 2 * f + om);
		nu = nu + 4 * sin(n - 2 * f);
		nu = nu - 4 * sin(-d + n);
		nu = nu - 4 * sin(-2 * d + m);
		nu = nu - 4 * sin(d);
		nu = nu + 3 * sin(n + 2 * f);
		nu = nu - 3 * sin(-2 * n + 2 * f + 2 * om);
		nu = nu - 3 * sin(-d - m - n);
		nu = nu - 3 * sin(m + n);
		nu = nu - 3 * sin(-m + n + 2 * f + 2 * om);
		nu = nu - 3 * sin(2 * d - m - n + 2 * f + 2 * om);
		nu = nu - 3 * sin(3 * n + 2 * f + 2 * om);
		nu = nu - 3 * sin(2 * d - m + 2 * f + 2 * om);

		double	nutation_en_longitude = nu / 10000;

		nu = (92025 + 8.9 * T_2000) * cos(om);
		nu = nu + (5736 - 3.1 * T_2000) * cos(-2 * d + 2 * f + 2 * om);
		nu = nu + (977 - 0.5 * T_2000) * cos(2 * f + 2 * om);
		nu = nu + (-895 + 0.5 * T_2000) * cos(2 * om);
		nu = nu + (54 - 0.1 * T_2000) * cos(m);
		nu = nu - 7 * cos(n);
		nu = nu + (224 - 0.6 * T_2000) * cos(-2 * d + m + 2 * f + 2 * om);
		nu = nu + 200 * cos(2 * f + om);
		nu = nu + (129 - 0.1 * T_2000) * cos(n + 2 * f + 2 * om);
		nu = nu + (-95 + 0.3 * T_2000) * cos(-2 * d - m + 2 * f + 2 * om);
		nu = nu - 70 * cos(-2 * d + 2 * f + om);
		nu = nu - 53 * cos(-n + 2 * f + 2 * om);
		nu = nu - 33 * cos(n + om);
		nu = nu + 26 * cos(2 * d - n + 2 * f + 2 * om);
		nu = nu + 32 * cos(-n + om);
		nu = nu + 27 * cos(n + 2 * f + om);
		nu = nu - 24 * cos(-2 * n + 2 * f + om);
		nu = nu + 16 * cos(2 * (d + f + om));
		nu = nu + 13 * cos(2 * (n + f + om));
		nu = nu - 12 * cos(-2 * d + n + 2 * f + 2 * om);
		nu = nu - 10 * cos(-n + 2 * f + om);
		nu = nu - 8 * cos(2 * d - n + om);
		nu = nu + 7 * cos(-2 * d + 2 * m + 2 * f + 2 * om);
		nu = nu + 9 * cos(m + om);
		nu = nu + 7 * cos(-2 * d + n + om);
		nu = nu + 6 * cos(-m + om);
		nu = nu + 5 * cos(2 * d - n + 2 * f + om);
		nu = nu + 3 * cos(2 * d + n + 2 * f + 2 * om);
		nu = nu - 3 * cos(m + 2 * f + 2 * om);
		nu = nu + 3 * cos(-m + 2 * f + 2 * om);
		nu = nu + 3 * cos(2 * d + 2 * f + om);
		nu = nu - 3 * cos(-2 * d + 2 * n + 2 * f + 2 * om);
		nu = nu - 3 * cos(-2 * d + n + 2 * f + om);
		nu = nu + 3 * cos(2 * d - 2 * n + om);
		nu = nu + 3 * cos(2 * d + om);
		nu = nu + 3 * cos(-2 * d - m + 2 * f + om);
		nu = nu + 3 * cos(-2 * d + om);
		nu = nu + 3 * cos(2 * n + 2 * f + om);

	double	nutation_en_obliquite = nu / 10000;

		obliquite = (23 + 26 / 60 + 21.448 / 3600 - 46.815 / 3600 * T_2000 - 0.00059 / 3600 * T2_2000 + 0.001813 / 3600 * T3_2000) / r2d + nutation_en_obliquite * ct;

		double	correct = 22639.55 * sin (n);
		correct = correct + 4586.43061 * sin (2 * d - n);
		correct = correct + 2369.91227 * sin (2 * d);
		correct = correct + 769.02326 * sin (2 * n);
		correct = correct + 211.65487 * sin (2 * d - 2 * n);
		correct = correct + 205.44315 * sin (2 * d - m - n);
		correct = correct + 191.95575 * sin (2 * d + n);
		correct = correct + 164.73458 * sin (2 * d - m);
		correct = correct + 55.17801 * sin (2 * d - 2 * f);
		correct = correct + 39.53393 * sin (n - 2 * f);
		correct = correct + 38.42974 * sin (4 * d - n);
		correct = correct + 36.12364 * sin (3 * n);
		correct = correct + 30.77247 * sin (4 * d - 2 * n);
		correct = correct + 17.95512 * sin (d + m);
		correct = correct + 14.53078 * sin (2 * d - m + n);
		correct = correct + 14.37964 * sin (2 * d + 2 * n);
		correct = correct + 13.89903 * sin (4 * d);
		correct = correct + 13.194 * sin (2 * d - 3 * n);
		correct = correct + 8.60582 * sin (2 * d - m - 2 * n);
		correct = correct + 8.05076 * sin (2 * d - 2 * m);
		correct = correct + 7.37173 * sin (2 * d - 2 * m - n);
		correct = correct + 4.37416 * sin (4 * d - m - n);
		correct = correct + 2.73198 * sin (4 * d - m - 2 * n);
		correct = correct + 2.48897 * sin (2 * d + m - 2 * n);
		correct = correct + 2.14619 * sin (2 * d - m - 2 * f);
		correct = correct + 1.97772 * sin (4 * d + n);
		correct = correct + 1.93367 * sin (4 * n);
		correct = correct + 1.87083 * sin (4 * d - m);
		correct = correct + 1.26186 * sin (d + m + n);
		correct = correct + 1.18682 * sin (4 * d - 3 * n);
		correct = correct + 1.17704 * sin (2 * d - m + 2 * n);
		correct = correct + 1.07773 * sin (d + m - n);
		correct = correct + 1.05949 * sin (2 * d + 3 * n);
		correct = correct + .94827 * sin (2 * d - 4 * n);
		correct = correct + .75173 * sin (2 * d - 2 * m + n);
		correct = correct + .57156 * sin (6 * d - 2 * n);
		correct = correct + .47842 * sin (2 * d - m - 3 * n);
		correct = correct + .42034 * sin (4 * f);
		correct = correct + .41342 * sin (m + 2 * f);
		correct = correct + .40423 * sin (3 * d);
		correct = correct + .39451 * sin (6 * d - n);
		correct = correct + .34966 * sin (d + m - 2 * n);
		correct = correct + .33983 * sin (2 * d - 3 * m);
		correct = correct + .30874 * sin (4 * d - 2 * m - n);
		correct = correct + .30157 * sin (m - n - 2 * f);
		correct = correct + .30086 * sin (4 * d - n - 2 * f);
		correct = correct + .29422 * sin (2 * d - 2 * m - 2 * n);
		correct = correct + .29255 * sin (6 * d - 3 * n);
		correct = correct + .28251 * sin (4 * d - m + n);
		correct = correct + .27377 * sin (3 * d + m - n);
		correct = correct + .26338 * sin (m + n + 2 * f);
		correct = correct + .25429 * sin (d + 2 * f);
		correct = correct + .24697 * sin (2 * d - 3 * m - n);
		correct = correct + .21853 * sin (4 * d + 2 * n);
		correct = correct + .17903 * sin (2 * d - n - 2 * f);
		correct = correct + .17624 * sin (2 * d + m - 3 * n);
		correct = correct + .15781 * sin (4 * d - 2 * m - 2 * n);
		correct = correct + .15227 * sin (4 * d - 2 * m);
		correct = correct + .1499 * sin (3 * d + m);
		correct = correct + .12616 * sin (6 * d);
		correct = correct + .111 * sin (5 * n);
		correct = correct + .09982 * sin (4 * d - m - 3 * n);
		correct = correct + .0932 * sin (2 * d - m + 3 * n);
		correct = correct + .09205 * sin (d + m + 2 * n);
		correct = correct + .09092 * sin (n + 4 * f);
		correct = correct + .09033 * sin (6 * d - m - 2 * n);
		correct = correct + .08472 * sin (2 * d + m + n - 2 * f);
		correct = correct + .07765 * sin (2 * d + 4 * n);
		correct = correct + .07501 * sin (m - 2 * f);
		correct = correct + .07142 * sin (6 * d - m - n);
		correct = correct + .0685 * sin (2 * d - 5 * n);
		correct = correct + .06742 * sin (2 * d + m - n + 2 * f);
		correct = correct + .06541 * sin (2 * d + m + 2 * f);
		correct = correct + .06507 * sin (3 * d - m);
		correct = correct + .06439 * sin (2 * d - 2 * m + 2 * n);
		correct = correct + .06314 * sin (2 * d - 2 * m - 2 * f);
		correct = correct + .05165 * sin (m - 2 * n - 2 * f);
		correct = correct + .0445 * sin (d + n + 2 * f);
		correct = correct + .04338 * sin (m + 2 * n + 2 * f);
		correct = correct + .04304 * sin (d - 2 * m);
		correct = correct + .039 * sin (6* d - m - 3 * n);
		correct = correct + .033 * sin (2 * d - 3 * m + n);
		correct = correct + .03274 * sin (4 * d - m + 2 * n);
		correct = correct + .02949 * sin (2 * d - m - 4 * n);
		correct = correct + .02682 * sin (4 * d + m - 3 * n);
		correct = correct + .02677 * sin (m + 2 * n - 2 * f);
		correct = correct + .0251 * sin (6 * d - m);
		correct = correct + .02429 * sin (m - 2 * n + 2 * f);
		correct = correct + .02411 * sin (4 * d - 2 * m + n);
		correct = correct + .02296 * sin (d + m - 3 * n);
		correct = correct + .02289 * sin (4 * d - m - n - 2 * f);
		correct = correct + .02285 * sin (6 * d + n);
		correct = correct + .02244 * sin (3 * d + m + n);
		correct = correct + .02149 * sin (4 * d + 3 * n);
		correct = correct + .01993 * sin (2 * d - n + 4 * f);
		correct = correct + .01819 * sin (2 * d + m - 4 * n);
		correct = correct + .01741 * sin (4 * d - 3 * m - n);
		correct = correct + .01605 * sin (2 * d + m + n + 2 * f);
		correct = correct + .01598 * sin (d - n + 2 * f);
		correct = correct + .01544 * sin (2 * d - 2 * m - 3 * n);
		correct = correct + .01376 * sin (6 * d - 4 * n);
		correct = correct + .01372 * sin (2 * d + 4 * f);
		correct = correct + .01331 * sin (2 * d - 4 * m);
		correct = correct + .01297 * sin (2 * n + 4 * f);
		correct = correct + .01215 * sin (3 * d - n + 2 * f);
		correct = correct + .00971 * sin (4 * d - 3 * m);
		correct = correct + .00965 * sin (2 * d - 3 * m - 2 * n);
		correct = correct + .00891 * sin (3 * d + m - 2 * f);
		correct = correct + .00889 * sin (2 * d + m + 2 * n - 2 * f);
		correct = correct + .00866 * sin (8 * d - 2 * n);
		correct = correct + .0084 * sin (8 * d - 3 * n);
		correct = correct + .00836 * sin (6 * d - 2 * m - 2 * n);
		correct = correct + .00812 * sin (2 * d - 4 * m - n);
		correct = correct + .00755 * sin (4 * d - 3 * m - 2 * n);
		correct = correct + .00744 * sin (6 * d - 2 * m - n);
		correct = correct + .0073 * sin (2 * d - m + 4 * n);
		correct = correct + .00679 * sin (d + m + 3 * n);
		correct = correct + .00666 * sin (4 * d - m - 2 * f);
		correct = correct + .00665 * sin (6 * n);
		correct = correct + .00662 * sin (4 * d - 2 * n - 2 * f);
		correct = correct + .00623 * sin (m - 3 * n - 2 * f);
		correct = correct + .00568 * sin (2 * d + 5 * n);
		correct = correct + .0056 * sin (4 * d - 2 * m - 3 * n);
		correct = correct + .0054 * sin (d + 2 * n + 2 * f);
		correct = correct + .00538 * sin (2 * d - 2 * m + 3 * n);
		correct = correct + .00526 * sin (m + 3 * n + 2 * f);
		correct = correct + .00519 * sin (2 * m + 2 * f);
		correct = correct + .00518 * sin (3 * d - 2 * m);
		correct = correct + .00515 * sin (2 * d + 2 * m - n + 2 * f);
		correct = correct + .00497 * sin (2 * d - 6 * n);
		correct = correct + .00477 * sin (6 * d - m + n);
		correct = correct + .00475 * sin (5 * d + m - n);
		correct = correct + .00473 * sin (2 * m - n - 2 * f);
		correct = correct + .00467 * sin (2 * d - 3 * n + 2 * f);
		correct = correct + .00455 * sin (8 * d - n);
		correct = correct + .00439 * sin (5 * d);
		correct = correct + .00392 * sin (5 * d + m - 2 * n);
		correct = correct + .00375 * sin (3 * d + 2 * f);
		correct = correct + .00364 * sin (6 * d - 2 * n - 2 * f);
		correct = correct + .00361 * sin (d + 2 * m - 2 * n);
		correct = correct + .00353 * sin (4 * d + m - n + 2 * f);
		correct = correct + .00344 * sin (2 * d + n + 4 * f);
		correct = correct + .00336 * sin (4 * d - m + 3 * n);
		correct = correct + .0033 * sin (3 * d - m + n);
		correct = correct + .00324 * sin (8 * d - 4 * n);
		correct = correct + .00318 * sin (6 * d + 2 * n);
		correct = correct + .00312 * sin (6 * d - 2 * m - 3 * n);
		correct = correct + .00298 * sin (3 * d - 2 * n + 2 * f);
		correct = correct + .00295 * sin (2 * d - 3 * m + 2 * n);
		correct = correct + .0029 * sin (4 * d - 2 * m + 2 * n);
		correct = correct + .00289 * sin (d - 2 * n - 2 * f);
		correct = correct + .00285 * sin (6 * d - 2 * m);
		correct = correct + .00282 * sin (2 * d - 2 * n + 4 * f);
		correct = correct + .0027 * sin (2 * m + n + 2 * f);
		correct = correct + .00262 * sin (2 * d + m + 2 * n + 2 * f);
		correct = correct + .00256 * sin (3 * d + m + 2 * n);
		correct = correct + .00254 * sin (d - 3 * m);
		correct = correct + .00229 * sin (d - 2 * m - n);
		correct = correct + .0022 * sin (4 * d + m - 2 * n + 2 * f);
		correct = correct + .00198 * sin (2 * d + m - 4 * f);
		correct = correct + .00198 * sin (4 * d + 4 * n);
		correct = correct + .00196 * sin (8 * d - m - 2 * n);
		correct = correct + .00186 * sin (4 * d + m + 2 * f);
		correct = correct + .00183 * sin (4 * d + m + n - 2 * f);
		correct = correct + .00181 * sin (5 * d + m);
		correct = correct + .00178 * sin (2 * d - m - 5 * n);
		correct = correct + .00176 * sin (6 * d - m - 4 * n);
		correct = correct + .00173 * sin (2 * d + m - 5 * n);
		correct = correct + .0017 * sin (8 * d - m - 3 * n);
		correct = correct + .00166 * sin (m + 3 * n - 2 * f);
		correct = correct + .00163 * sin (2 * d - 3 * m - 2 * f);
		correct = correct + .0016 * sin (4 * d - 3 * m + n);
		correct = correct + .00155 * sin (d - m + 2 * f);
		correct = correct + .00155 * sin (d + m - 4 * n);
		correct = correct + .00153 * sin (3 * n + 4 * f);
		correct = correct + .00139 * sin (8 * d);
		correct = correct + .00133 * sin (2 * d - 4 * m + n);
		correct = correct + .00123 * sin (d - 4 * f);
		correct = correct + .00116 * sin (3 * d + m - n - 2 * f);
		correct = correct + .00112 * sin (8 * d - m - n);
		correct = correct + .00108 * sin (4 * d - 2 * m - n - 2 * f);
		correct = correct + .00106 * sin (m - 3 * n + 2 * f);
		correct = correct + .00102 * sin (5 * d - m);
		correct = correct + .001 * sin (2 * m - 2 * n - 2 * f);
		correct = correct + .00096 * sin (2 * d + 2 * m + 2 * f);

		correct = correct - 666.44186 * sin (m);
		correct = correct - 411.60287 * sin (2 * f);
		correct = correct - 147.32654 * sin (m - n);
		correct = correct - 124.98806 * sin (d);
		correct = correct - 109.38419 * sin (m + n);
		correct = correct - 45.10032 * sin (n + 2 * f);
		correct = correct - 28.3981 * sin (2 * d + m - n);
		correct = correct - 24.3591 * sin (2 * d + m);
		correct = correct - 18.58467 * sin (d - n);
		correct = correct - 9.67938 * sin (m - 2 * n);
		correct = correct - 9.36601 * sin (2 * d - n + 2 * f);
		correct = correct - 8.45308 * sin (d + n);
		correct = correct - 7.63041 * sin (m + 2 * n);
		correct = correct - 7.44804 * sin (2 * m);
		correct = correct - 6.38325 * sin (2 * d + n - 2 * f);
		correct = correct - 5.7417 * sin (2 * d + 2 * f);
		correct = correct - 3.99767 * sin (2 * n + 2 * f);
		correct = correct - 3.20968 * sin (3 * d - n);
		correct = correct - 2.91464 * sin (2 * d + m + n);
		correct = correct - 2.56813 * sin (2 * m - n);
		correct = correct - 2.52138 * sin (2 * d + 2 * m - n);
		correct = correct - 1.75296 * sin (d - 2 * n);
		correct = correct - 1.43724 * sin (2 * d + m - 2 * f);
		correct = correct - 1.37259 * sin (2 * n - 2 * f);
		correct = correct - 1.22412 * sin (3 * d - 2 * n);
		correct = correct - 1.16177 * sin (2 * m + n);
		correct = correct - .99023 * sin (2 * d + n + 2 * f);
		correct = correct - .6694 * sin (m - 3 * n);
		correct = correct - .63523 * sin (4 * d + m - n);
		correct = correct - .58399 * sin (d + 2 * n);
		correct = correct - .58332 * sin (d - 2 * f);
		correct = correct - .56065 * sin (2 * d - 2 * n - 2 * f);
		correct = correct - .55694 * sin (d - m);
		correct = correct - .54594 * sin (m + 3 * n);
		correct = correct - .53572 * sin (2* d - 2 * n + 2 * f);
		correct = correct - .4538 * sin (2 * d + 2 * n - 2 * f);
		correct = correct - .42624 * sin (2 * d - m - n + 2 * f);
		correct = correct - .38215 * sin (2 * d - m + 2 * f);
		correct = correct - .37453 * sin (2 * d - m + n - 2 * f);
		correct = correct - .35759 * sin (4 * d + m - 2 * n);
		correct = correct - .32866 * sin (3 * n + 2 * f);
		correct = correct - .29023 * sin (2 * d + m + 2 * n);
		correct = correct - .28911 * sin (4 * d + m);
		correct = correct - .25304 * sin (3 * d - 2 * f);
		correct = correct - .2499 * sin (2 * d + 2 * m - 2 * n);
		correct = correct - .23141 * sin (3 * d - m - n);
		correct = correct - .20134 * sin (4 * d - n + 2 * f);
		correct = correct - .19311 * sin (2 * m - 2 * n);
		correct = correct - .18576 * sin (2 * d + 2 * m);
		correct = correct - .16977 * sin (4 * d - 2 * n + 2 * f);
		correct = correct - .13636 * sin (d - m - n);
		correct = correct - .12812 * sin (d - 3 * n);
		correct = correct - .12386 * sin (2 * d + 2 * n + 2 * f);
		correct = correct - .12073 * sin (d - m + n);
		correct = correct - .10136 * sin (3 * m);
		correct = correct - .09154 * sin (2 * d - 3 * n - 2 * f);
		correct = correct - .085 * sin (4 * d + 2 * f);
		correct = correct - .08311 * sin (3 * d - m - 2 * n);
		correct = correct - .08282 * sin (m + n - 2 * f);
		correct = correct - .08049 * sin (m - n + 2 * f);
		correct = correct - .08019 * sin (n - 4 * f);
		correct = correct - .07518 * sin (2 * d - 4 * f);
		correct = correct - .07373 * sin (2 * d - m + n + 2 * f);
		correct = correct - .06601 * sin (4 * d + n - 2 * f);
		correct = correct - .06513 * sin (2 * m + 2 * n);
		correct = correct - .06103 * sin (2 * d - m - n - 2 * f);
		correct = correct - .05725 * sin (5 * d - 2 * n);
		correct = correct - .05684 * sin (3 * n - 2 * f);
		correct = correct - .05142 * sin (3 * m - n);
		correct = correct - .0507 * sin (4 * d + m + n);
		correct = correct - .04702 * sin (m - 4 * n);
		correct = correct - .04442 * sin (3 * d - 3 * n);
		correct = correct - .04189 * sin (3 * d + m - 2 * n);
		correct = correct - .04074 * sin (d + 3 * n);
		correct = correct - .04012 * sin (d + n - 2 * f);
		correct = correct - .03968 * sin (d + 2 * m);
		correct = correct - .03947 * sin (m + 4 * n);
		correct = correct - .03587 * sin (d + m + 2 * f);
		correct = correct - .03514 * sin (4 * d + 2 * m - 2 * n);
		correct = correct - .03336 * sin (2 * d + 3 * n - 2 * f);
		correct = correct - .02979 * sin (3 * d - n - 2 * f);
		correct = correct - .02887 * sin (2 * d - m + 2 * n - 2 * f);
		correct = correct - .02804 * sin (2 * d - m - 2 * n - 2 * f);
		correct = correct - .02676 * sin (2 * d + m + 3 * n);
		correct = correct - .02602 * sin (4 * n + 2 * f);
		correct = correct - .02391 * sin (4 * d - 2 * f);
		correct = correct - .02379 * sin (d - n - 2 * f);
		correct = correct - .02349 * sin (2 * d + 2 * m - 2 * f);
		correct = correct - .02273 * sin (4 * d - m - n + 2 * f);
		correct = correct - .02171 * sin (4 * d + 2 * m - n);
		correct = correct - .02157 * sin (2 * d - m - 2 * n + 2 * f);
		correct = correct - .01948 * sin (3 * d - m - 2 * f);
		correct = correct - .01875 * sin (4 * d + m - n - 2 * f);
		correct = correct - .01816 * sin (2 * d - 2 * m + 2 * f);
		correct = correct - .01796 * sin (3 * m + n);
		correct = correct - .01781 * sin (4 * d + n + 2 * f);
		correct = correct - .01686 * sin (5 * d - 3 * n);
		correct = correct - .01644 * sin (2 * d - 2 * m + n - 2 * f);
		correct = correct - .01541 * sin (2 * d - 2 * m - n + 2 * f);
		correct = correct - .01533 * sin (4 * d - m - 2 * n + 2 * f);
		correct = correct - .01514 * sin (2 * m - 3 * n);
		correct = correct - .01483 * sin (d - m + 2 * n);
		correct = correct - .0135 * sin (5 * d - n);
		correct = correct - .01343 * sin (2 * d + 2 * m + n);
		correct = correct - .01332 * sin (2 * d + 3 * n + 2 * f);
		correct = correct - .01282 * sin (6 * d + m - 2 * n);
		correct = correct - .01281 * sin (d - m - 2 * f);
		correct = correct - .01182 * sin (3 * d - 2 * m - n);
		correct = correct - .01114 * sin (4 * d - m + 2 * f);
		correct = correct - .01077 * sin (2 * d - 4 * n - 2 * f);
		correct = correct - .01064 * sin (6 * d + m - n);
		correct = correct - .01062 * sin (3 * d + n - 2 * f);
		correct = correct - .01007 * sin (2 * d - m + 2 * n + 2 * f);
		correct = correct - .0098 * sin (4 * d + 2 * n - 2 * f);
		correct = correct - .00955 * sin (d - 4 * n);
		correct = correct - .00944 * sin (2 * d + 2 * m - 3 * n);
		correct = correct - .00934 * sin (4 * d - 3 * n + 2 * f);
		correct = correct - .0085 * sin (2 * d - n - 4 * f);
		correct = correct - .00849 * sin (d + 2 * m + n);
		correct = correct - .00732 * sin (4 * d - m + n - 2 * f);
		correct = correct - .00694 * sin (d - m - 2 * n);
		correct = correct - .00693 * sin (5 * d - m - 2 * n);
		correct = correct - .00668 * sin (4 * d + m + 2 * n);
		correct = correct - .00659 * sin (d + m + n + 2 * f);
		correct = correct - .00654 * sin (2 * d + 2 * m - n - 2 * f);
		correct = correct - .00623 * sin (3 * d + m - 3 * n);
		correct = correct - .00509 * sin (6 * d - 2 * n + 2 * f);
		correct = correct - .00478 * sin (6 * d + m - 3 * n);
		correct = correct - .00434 * sin (2 * d - 2 * m - n - 2 * f);
		correct = correct - .00431 * sin (4 * d - 5 * n);
		correct = correct - .00416 * sin (3 * m - 2 * n);
		correct = correct - .00399 * sin (3 * d - 2 * m - 2 * n);
		correct = correct - .00396 * sin (6 * d + m);
		correct = correct - .00389 * sin (3 * d + 2 * n);
		correct = correct - .00378 * sin (2 * d - 2 * m + n + 2 * f);
		correct = correct - .00369 * sin (4 * d + 2 * m - 3 * n);
		correct = correct - .00365 * sin (2 * d - m - 3 * n - 2 * f);
		correct = correct - .00359 * sin (6 * d - n + 2 * f);
		correct = correct - .00355 * sin (2 * m - 2 * f);
		correct = correct - .00354 * sin (4 * n - 2 * f);
		correct = correct - .00346 * sin (2 * d + m - 2 * n - 2 * f);
		correct = correct - .00341 * sin (2 * m + 3 * n);
		correct = correct - .00335 * sin (5 * d - n - 2 * f);
		correct = correct - .00332 * sin (m - 5 * n);
		correct = correct - .003 * sin ( d + 2 * m - n);
		correct = correct - .00297 * sin (3 * d - m - 3 * n);
		correct = correct - .00287 * sin (m + 5 * n);
		correct = correct - .00287 * sin (6 * d - 3 * n + 2 * f);
		correct = correct - .00286 * sin (2 * d - m - 4 * f);
		correct = correct - .00285 * sin (d + 4 * n);
		correct = correct - .00274 * sin (4 * d + 2 * n + 2 * f);
		correct = correct - .00251 * sin (4 * d - m + n + 2 * f);
		correct = correct - .00247 * sin (2 * d + 4 * n - 2 * f);
		correct = correct - .00236 * sin (2 * d + m + 4 * n);
		correct = correct - .00232 * sin (2 * d - m + 3 * n - 2 * f);
		correct = correct - .00228 * sin (2 * d + m - n - 2 * f);
		correct = correct - .00214 * sin (6 * d - 2 * f);
		correct = correct - .00212 * sin (d - m + n - 2 * f);
		correct = correct - .00208 * sin (4 * d + 2 * m);
		correct = correct - .00201 * sin (5 * n + 2 * f);
		correct = correct - .002 * sin (2 * d + 2 * m + n - 2 * f);
		correct = correct - .00191 * sin (3 * d + 2 * m);
		correct = correct - .00189 * sin (3 * d - m - n - 2 * f);
		correct = correct - .00189 * sin (5 * d - m - 3 * n);
		correct = correct - .00188 * sin (2 * d + 3 * m - n);
		correct = correct - .00174 * sin (3 * d - 4 * n);
		correct = correct - .0016 * sin (4 * d - 2 * m - n + 2 * f);
		correct = correct - .00157 * sin (d + m + n - 2 * f);
		correct = correct - .00154 * sin (5 * d - m - n);
		correct = correct - .00149 * sin (d - m + 3 * n);
		correct = correct - .00142 * sin (d - 2 * n + 2 * f);
		correct = correct - .00138 * sin (3 * d + m - n + 2 * f);
		correct = correct - .00137 * sin (5 * d - 2 * f);
		correct = correct - .00133 * sin (2 * d - 2 * m + 2 * n - 2 * f);
		correct = correct - .00132 * sin (6 * d + 2 * f);
		correct = correct - .00131 * sin (2 * d + 4 * n + 2 * f);
		correct = correct - .00128 * sin (4 * m);
		correct = correct - .00127 * sin (3 * d + 2 * m - n);
		correct = correct - .00121 * sin (4 * d - m + 2 * n - 2 * f);
		correct = correct - .00119 * sin (2 * m - 4 * n);
		correct = correct - .00117 * sin (2 * d - m + 3 * n + 2 * f);
		correct = correct - .00116 * sin (2 * d + m - 3 * n - 2 * f);
		correct = correct - .00111 * sin (2 * d - 2 * m - 2 * n - 2 * f);
		correct = correct - .00111 * sin (2 * d - 5 * n - 2 * f);
		correct = correct - .00109 * sin (4 * d + 3 * n - 2 * f);
		correct = correct - .00108 * sin (4 * m - n);
		correct = correct - .00102 * sin (d + 2 * m + 2 * n);
		correct = correct - .00102 * sin (3 * d - 2 * m - 2 * f);
		correct = correct - .001 * sin (d - m - n - 2 * f);
		correct = correct - .00098 * sin (7 * d - 3 * n);

		correct = correct + 14.2488 * sin (18 * venus - 16 * longitude_moyenne_terre - n + 26.54261 / r2d);
		correct = correct + 1.1431 * sin (2 * longitude_moyenne_terre - 2 * jupiter + 2 * d - n + 180.11977 / r2d);
		correct = correct + 0.9011 * sin (4 * longitude_moyenne_terre - 8 * mars + 3 * jupiter + 285.98707 / r2d);
		correct = correct + 0.8216 * sin (venus - longitude_moyenne_terre + 180.00988 / r2d);
		correct = correct + 0.7881 * sin (18 * venus - 16 * longitude_moyenne_terre - 2 * n + 26.54324 / r2d);
		correct = correct + 0.7393 * sin (18 * venus - 16 * longitude_moyenne_terre + 26.54560 / r2d);
		correct = correct + 0.6437 * sin (3 * venus - 3 * longitude_moyenne_terre + 2 * d - n + 179.98144 / r2d);
		correct = correct + 0.6388 * sin (longitude_moyenne_terre - jupiter + 1.22890 / r2d);
		correct = correct + 0.5634 * sin (10 * venus - 3 * longitude_moyenne_terre - n + 333.30551 / r2d);
		correct = correct + 0.4453 * sin (2 * longitude_moyenne_terre - 3 * jupiter + 2 * d - n + 10.07001 / r2d);
		correct = correct + 0.3436 * sin (2 * venus - 3 * longitude_moyenne_terre + 269.95393 / r2d);
		correct = correct + 0.3246 * sin (longitude_moyenne_terre - 2 * mars + 318.13776 / r2d);
		correct = correct + 0.3016 * sin (2 * venus - 2 * longitude_moyenne_terre + 0.20448 / r2d);

		correct = correct + 7.06304 * sin (dzeta_lune - f + 0.00094 / r2d);
		correct = correct + 0.49331 * sin (dzeta_lune + petit_l - f + 0.00127 / r2d);
		correct = correct + 0.49141 * sin (dzeta_lune - petit_l - f + 0.00127 / r2d);
		correct = correct + 0.36061 * sin (dzeta_lune + f + 0.00071 / r2d);
		correct = correct + 0.09642 * sin (dzeta_lune + 2 * d - f + 0.0009 / r2d);
		correct = correct + 0.06569 * sin (dzeta_lune - 2 * d - f + 0.001 / r2d);
		correct = correct + 0.06456 * sin (dzeta_lune + 2 * d - petit_l - f + 0.00042 / r2d);
		correct = correct + 0.05036 * sin (dzeta_lune - petit_l + f + 0.00051 / r2d);
		correct = correct + 0.04962 * sin (dzeta_lune - 2*d + petit_l - f + 0.00029 / r2d);
		correct = correct + 0.04746 * sin (dzeta_lune - 2 * d + f + 0.00076 / r2d);
		correct = correct + 0.03838 * sin (dzeta_lune + petit_l + f + 0.0007 / r2d);
		correct = correct + 0.03638 * sin (2 * dzeta_lune - 2 * f + 180 / r2d);
		correct = correct + 0.03402 * sin (dzeta_lune + 2 * petit_l - f + 0.00126 / r2d);
		correct = correct + 0.03279 * sin (dzeta_lune - 2 * petit_l - f + 0.00128 / r2d);
		correct = correct + 0.02206 * sin (2 * d - petit_l);
		correct = correct + 0.01492 * sin (dzeta_lune - 3 * f + 180.00086 / r2d);
		correct = correct + 0.01234 * sin (dzeta_lune + 2 * d + petit_l - f + 0.00102 / r2d);

		l = l + correct / 3600 / r2d + nutation_en_longitude * ct;
		longitude = l;
		double longitude_lune = l;

		l1 = l * r2d;

		d1 = ((l1 - floor(l1)) * 60);
		d2 = ((d1 - floor(d1)) * 60);

		correct = 18461.4 * sin (f);
		correct = correct -6.29664 * sin (3 * f);
		correct = correct + 2.79871 * sin (n - 3 * f);
		correct = correct + 999.70079 * sin (n - f);
		correct = correct + 1010.1743 * sin (n + f);
		correct = correct - 1.01941 * sin (n + 3 * f);
		correct = correct - .13035 * sin (2 * n -  3 * f);
		correct = correct + 31.75985 * sin (2 * n - f);
		correct = correct + 61.91229 * sin (2 * n + f);
		correct = correct - .11787 * sin (2 * n +  3 * f);
		correct = correct + 1.58131 * sin (3 * n - f);
		correct = correct + 3.98407 * sin (3 * n + f);
		correct = correct - .01181 * sin (3 * n +  3 * f);
		correct = correct + .09157 * sin (4 * n - f);
		correct = correct + .26325 * sin (4 * n + f);
		correct = correct + .01768 * sin (5 * n + f);
		correct = correct - .07479 * sin (m -  3 * n - f);
		correct = correct - .02365 * sin (m -  3 * n + f);
		correct = correct - .79322 * sin (m -  2 * n - f);
		correct = correct - .30129 * sin (m -  2 * n + f);
		correct = correct - 6.73173 * sin (m - n - f);
		correct = correct - 5.6326 * sin (m - n + f);
		correct = correct - 4.83983 * sin (m - f);
		correct = correct - 6.46036 * sin (m + f);
		correct = correct + .01157 * sin (m +  3 * f);
		correct = correct - 5.07614 * sin (m + n - f);
		correct = correct - 5.31151 * sin (m + n + f);
		correct = correct - .31292 * sin (m +  2 * n - f);
		correct = correct - .63884 * sin (m +  2 * n + f);
		correct = correct - .02419 * sin (m +  3 * n - f);
		correct = correct - .06176 * sin (m +  3 * n + f);
		correct = correct - .01571 * sin (2 * m -  2 * n - f);
		correct = correct - .11335 * sin (2 * m - n - f);
		correct = correct - .09511 * sin (2 * m - n + f);
		correct = correct - .01801 * sin (2 * m - f);
		correct = correct - .05729 * sin (2 * m + f);
		correct = correct - .06187 * sin (2 * m + n - f);
		correct = correct - .05504 * sin (2 * m + n + f);
		correct = correct + .01031 * sin (d - m - n - f);
		correct = correct - .01346 * sin (d - m - f);
		correct = correct - .01829 * sin (d - m + f);
		correct = correct - .02012 * sin (d - m + n - f);
		correct = correct - .01255 * sin (d -  3 * n - f);
		correct = correct - .10964 * sin (d -  2 * n - f);
		correct = correct - .07846 * sin (d -  2 * n + f);
		correct = correct - .42989 * sin (d - n - f);
		correct = correct + .13928 * sin (d - n + f);
		correct = correct - .03226 * sin (d -  3 * f);
		correct = correct - 4.80578 * sin (d - f);
		correct = correct - 5.36844 * sin (d + f);
		correct = correct - .58893 * sin (d + n - f);
		correct = correct - .66741 * sin (d + n + f);
		correct = correct - .03636 * sin (d +  2 * n - f);
		correct = correct - .06383 * sin (d +  2 * n + f);
		correct = correct + .01597 * sin (d + m -  2 * n - f);
		correct = correct + .0168 * sin (d + m -  2 * n + f);
		correct = correct - .0559 * sin (d + m - n + f);
		correct = correct + .80426 * sin (d + m - f);
		correct = correct + .80263 * sin (d + m + f);
		correct = correct + .03465 * sin (d + m + n - f);
		correct = correct + .10176 * sin (d + m + n + f);
		correct = correct + .01016 * sin (d + m +  2 * n + f);
		correct = correct + .01042 * sin (2 * d -  3 * m - n + f);
		correct = correct + .03647 * sin (2 * d -  3 * m - f);
		correct = correct + .01603 * sin (2 * d -  3 * m + f);
		correct = correct + .02285 * sin (2 * d -  2 * m -  2 * n - f);
		correct = correct + .26865 * sin (2 * d -  2 * m - n - f);
		correct = correct + .31474 * sin (2 * d -  2 * m - n + f);
		correct = correct + 1.08587 * sin (2 * d -  2 * m - f);
		correct = correct + .38353 * sin (2 * d -  2 * m + f);
		correct = correct + .06915 * sin (2 * d -  2 * m + n - f);
		correct = correct + .05848 * sin (2 * d -  2 * m + n + f);
		correct = correct + .05502 * sin (2 * d - m -  3 * n - f);
		correct = correct + .65025 * sin (2 * d - m -  2 * n - f);
		correct = correct - .06208 * sin (2 * d - m -  2 * n + f);
		correct = correct + .01034 * sin (2 * d - m - n -  3 * f);
		correct = correct + 7.43488 * sin (2 * d - m - n - f);
		correct = correct + 8.86853 * sin (2 * d - m - n + f);
		correct = correct - .01177 * sin (2 * d - m - n +  3 * f);
		correct = correct + .08815 * sin (2 * d - m -  3 * f);
		correct = correct + 29.57794 * sin (2 * d - m - f);
		correct = correct + 7.95891 * sin (2 * d - m + f);
		correct = correct - .01669 * sin (2 * d - m + n -  3 * f);
		correct = correct + 1.76606 * sin (2 * d - m + n - f);
		correct = correct + 1.13466 * sin (2 * d - m + n + f);
		correct = correct + .12897 * sin (2 * d - m +  2 * n - f);
		correct = correct + .12387 * sin (2 * d - m +  2 * n + f);
		correct = correct + .01211 * sin (2 * d - m +  3 * n + f);
		correct = correct + .01127 * sin (2 * d - 5 * n - f);
		correct = correct + .13381 * sin (2 * d - 4 * n - f);
		correct = correct + .02496 * sin (2 * d -  4 * n + f);
		correct = correct + 1.51564 * sin (2 * d -  3 * n - f);
		correct = correct + .25408 * sin (2 * d -  3 * n + f);
		correct = correct + .02045 * sin (2 * d -  2 * n -  3 * f);
		correct = correct + 15.56635 * sin (2 * d -  2 * n - f);
		correct = correct - 1.62443 * sin (2 * d -  2 * n + f);
		correct = correct - .06561 * sin (2 * d -  2 * n +  3 * f);
		correct = correct + .32907 * sin (2 * d - n -  3 * f);
		correct = correct + 166.57528 * sin (2 * d - n - f);
		correct = correct + 199.48515 * sin (2 * d - n + f);
		correct = correct - .24484 * sin (2 * d - n +  3 * f);
		correct = correct + 2.18637 * sin (2 * d -  3 * f);
		correct = correct + 623.65783 * sin (2 * d - f);
		correct = correct + 117.26161 * sin (2 * d + f);
		correct = correct - .14453 * sin (2 * d +  3 * f);
		correct = correct - .29116 * sin (2 * d + n -  3 * f);
		correct = correct + 33.35743 * sin (2 * d + n - f);
		correct = correct + 15.12165 * sin (2 * d + n + f);
		correct = correct - .03038 * sin (2 * d + n +  3 * f);
		correct = correct + 2.14618 * sin (2 * d +  2 * n - f);
		correct = correct + 1.51976 * sin (2 * d +  2 * n + f);
		correct = correct + .14642 * sin (2 * d +  3 * n - f);
		correct = correct + .13795 * sin (2 * d +  3 * n + f);
		correct = correct + .01027 * sin (2 * d +  4 * n - f);
		correct = correct + .01186 * sin (2 * d +  4 * n + f);
		correct = correct + .01818 * sin (2 * d + m -  3 * n - f);
		correct = correct + .07913 * sin (2 * d + m -  2 * n - f);
		correct = correct + .05429 * sin (2 * d + m -  2 * n + f);
		correct = correct - .79105 * sin (2 * d + m - n - f);
		correct = correct - 1.31788 * sin (2 * d + m - n + f);
		correct = correct - .05457 * sin (2 * d + m -  3 * f);
		correct = correct - 12.0947 * sin (2 * d + m - f);
		correct = correct - 1.26433 * sin (2 * d + m + f);
		correct = correct - .82275 * sin (2 * d + m + n - f);
		correct = correct - .23702 * sin (2 * d + m + n + f);
		correct = correct - .06283 * sin (2 * d + m +  2 * n - f);
		correct = correct - .03142 * sin (2 * d + m +  2 * n + f);
		correct = correct - .01262 * sin (2 * d +  2 * m -  2 * n - f);
		correct = correct - .10535 * sin (2 * d +  2 * m - n - f);
		correct = correct - .1133 * sin (2 * d +  2 * m - n + f);
		correct = correct - .13415 * sin (2 * d +  2 * m - f);
		correct = correct - .01482 * sin (2 * d +  2 * m + f);
		correct = correct - .02104 * sin (3 * d - m - n - f);
		correct = correct - .01356 * sin (3 * d - m - n + f);
		correct = correct - .02572 * sin (3 * d - m - f);
		correct = correct - .03941 * sin (3 * d -  2 * n - f);
		correct = correct - .04852 * sin (3 * d -  2 * n + f);
		correct = correct - .30517 * sin (3 * d - n - f);
		correct = correct - .20593 * sin (3 * d - n + f);
		correct = correct - .01009 * sin (3 * d -  3 * f);
		correct = correct - .35183 * sin (3 * d - f);
		correct = correct - .0284 * sin (3 * d + f);
		correct = correct - .03611 * sin (3 * d + n - f);
		correct = correct + .01321 * sin (3 * d + m - n - f);
		correct = correct + .02083 * sin (3 * d + m - n + f);
		correct = correct + .03436 * sin (3 * d + m - f);
		correct = correct + .01351 * sin (3 * d + m + f);
		correct = correct + .0123 * sin (4 * d -  2 * m -  2 * n + f);
		correct = correct + .03462 * sin (4 * d -  2 * m - n - f);
		correct = correct + .0238 * sin (4 * d -  2 * m - n + f);
		correct = correct + .02899 * sin (4 * d -  2 * m - f);
		correct = correct + .0127 * sin (4 * d -  2 * m + f);
		correct = correct + .05251 * sin (4 * d - m -  2 * n - f);
		correct = correct + .21376 * sin (4 * d - m -  2 * n + f);
		correct = correct + .5958 * sin (4 * d - m - n - f);
		correct = correct + .33882 * sin (4 * d - m - n + f);
		correct = correct + .41496 * sin (4 * d - m - f);
		correct = correct + .15791 * sin (4 * d - m + f);
		correct = correct + .05686 * sin (4 * d - m + n - f);
		correct = correct + .03009 * sin (4 * d - m + n + f);
		correct = correct + .02174 * sin (4 * d -  3 * n + f);
		correct = correct + .63371 * sin (4 * d -  2 * n - f);
		correct = correct + 2.41389 * sin (4 * d -  2 * n + f);
		correct = correct + 6.57962 * sin (4 * d - n - f);
		correct = correct + 2.9985 * sin (4 * d - n + f);
		correct = correct + .06257 * sin (4 * d -  3 * f);
		correct = correct + 3.67449 * sin (4 * d - f);
		correct = correct + 1.19188 * sin (4 * d + f);
		correct = correct + .47338 * sin (4 * d + n - f);
		correct = correct + .21259 * sin (4 * d + n + f);
		correct = correct + .04834 * sin (4 * d +  2 * n - f);
		correct = correct + .02828 * sin (4 * d +  2 * n + f);
		correct = correct - .02957 * sin (4 * d + m -  2 * n + f);
		correct = correct - .17191 * sin (4 * d + m - n - f);
		correct = correct - .05097 * sin (4 * d + m - n + f);
		correct = correct - .11308 * sin (4 * d + m - f);
		correct = correct - .02549 * sin (4 * d + m + f);
		correct = correct - .01692 * sin (4 * d + m + n - f);
		correct = correct - .01049 * sin (5 * d - n - f);
		correct = correct + .01091 * sin (6 * d - m -  2 * n - f);
		correct = correct + .01486 * sin (6 * d - m - n - f);
		correct = correct + .03118 * sin (6 * d -  3 * n + f);
		correct = correct + .08096 * sin (6 * d -  2 * n - f);
		correct = correct + .05963 * sin (6 * d -  2 * n + f);
		correct = correct + .09403 * sin (6 * d - n - f);
		correct = correct + .04217 * sin (6 * d - n + f);
		correct = correct + .03674 * sin (6 * d - f);
		correct = correct + .01465 * sin (6 * d + f);

		correct = correct + 8.045 * sin (dzeta_lune + 180 / r2d);
		correct = correct + 0.416 * sin (dzeta_lune + petit_l + 180 / r2d);
		correct = correct + 0.456 * sin (dzeta_lune - petit_l);
		correct = correct + 0.326 * sin (dzeta_lune - 2 * f);

		correct = correct + 0.63 * sin (18 * venus - 16 * longitude_moyenne_terre - petit_l + f + 26.54 / r2d);
		correct = correct + 0.63 * sin (18 * venus - 16 * longitude_moyenne_terre-petit_l - f + 26.54 / r2d);
		correct = correct + 0.14 * sin (longitude_moyenne_terre + d + 291.98 / r2d);
		correct = correct + 0.07 * sin (18 * venus - 16 * longitude_moyenne_terre - 2 * petit_l - f + 26.54 / r2d);
		correct = correct + 0.067 * sin (18 * venus - 16 * longitude_moyenne_terre + f + 26.54 / r2d);
		correct = correct + 0.067 * sin (5 * venus - 6 * longitude_moyenne_terre + 2 * d - f + 272.3 / r2d);

		correct = correct + 1.375 * sin (longitude_moyenne_terre + d + 275.13 / r2d);
		correct = correct + 0.078 * sin (longitude_moyenne_terre + d - petit_l + 95.13 / r2d);

		correct = correct - 0.00001754 * 3600 / r2d * sin(183.3 / r2d + 483202 / r2d * T_2000);

		double latitude = correct / 3600 / r2d;
		b = latitude;
		double latitude_lune = b;

		b1 = abs(b * r2d);
		d1 = ((b1 - floor(b1)) * 60);
		d2 = ((d1 - floor(d1)) * 60);
		 
		if (b < 0) {signe = "-";}
		else {signe = "+";}

	
		double	p = 385000.56 - 3.14837 * cos(2 * f) + 79.66183 * cos(n - 2 * f);
		p = p - 20905.32206 * cos(n) - 0.10326 * cos(n + 2 * f);
		p = p - 4.42124 * cos(2 * n - 2 * f) - 569.92332 * cos(2 * n);
		p = p - 23.21032 * cos(3 * n) - 1.11693 * cos(4 * n);
		p = p - 0.42242 * cos(m - 3 * n) - 7.00293 * cos(m - 2 * n);
		p = p - 129.62476 * cos(m - n) + 0.33465 * cos(m - n + 2 * f);
		p = p - 0.18568 * cos(m - 2 * f) + 48.89 * cos(m);
		p = p - 0.15803 * cos(m + 2 * f) - 0.2481 * cos(m + n - 2 * f);
		p = p + 104.75896 * cos(m + n) + 5.75105 * cos(m + 2 * n);
		p = p + 0.35509 * cos(m + 3 * n) - 0.13618 * cos(2 * m - 2 * n);
		p = p - 2.11728 * cos(2 * m - n) + 1.06575 * cos(2 * m);
		p = p + 1.16562 * cos(2 * m + n) + 0.1141 * cos(d - m - n);
		p = p + 0.49757 * cos(d - m) + 0.10998 * cos(d - m + n);
		p = p - 1.73852 * cos(d - 2 * n) - 8.37909 * cos(d - n);
		p = p - 0.79564 * cos(d - 2 * f) + 108.74265 * cos(d);
		p = p + 6.32199 * cos(d + n) + 0.37852 * cos(d + 2 * n);
		p = p + 0.33226 * cos(d + m - 2 * n) + 0.85127 * cos(d + m - n);
		p = p - 16.67533 * cos(d + m) - 0.93335 * cos(d + m + n);
		p = p - 0.14808 * cos(2 * d - 3 * m - n) - 0.41076 * cos(2 * d - 3 * m);
		p = p + 0.34304 * cos(2 * d - 2 * m - 2 * n) - 4.95049 * cos(2 * d - 2 * m - n);
		p = p - 9.88519 * cos(2 * d - 2 * m) - 0.65758 * cos(2 * d - 2 * m + n);
		p = p + 0.49506 * cos(2 * d - m - 3 * n) + 10.05654 * cos(2 * d - m - 2 * n);
		p = p + 0.32336 * cos(2 * d - m - n - 2 * f) - 152.14314 * cos(2 * d - m - n);
		p = p + 0.657 * cos(2 * d - m - 2 * f) - 204.59357 * cos(2 * d - m);
		p = p + 0.20942 * cos(2 * d - m + n - 2 * f) - 12.83185 * cos(2 * d - m + n);
		p = p - 0.84883 * cos(2 * d - m + 2 * n) + 0.77854 * cos(2 * d - 4 * n);
		p = p + 14.40262 * cos(2 * d - 3 * n) + 0.47263 * cos(2 * d - 2 * n - 2 * f);
		p = p + 246.15768 * cos(2 * d - 2 * n) + 0.77405 * cos(2 * d - 2 * n + 2 * f);
		p = p + 8.7517 * cos(2 * d - n - 2 * f) - 3699.10468 * cos(2 * d - n);
		p = p + 0.59633 * cos(2 * d - n + 2 * f) + 10.32129 * cos(2 * d - 2 * f);
		p = p - 2955.9665 * cos(2 * d) + 4.13118 * cos(2 * d + n - 2 * f);
		p = p - 170.73274 * cos(2 * d + n) + 0.28399 * cos(2 * d + 2 * n - 2 * f);
		p = p - 10.44472 * cos(2 * d + 2 * n) - 0.66968 * cos(2 * d + 3 * n);
		p = p + 0.16858 * cos(2 * d + m - 3 * n) + 0.14368 * cos(2 * d + m - 2 * n);
		p = p + 24.20935 * cos(2 * d + m - n) - 0.13572 * cos(2 * d + m - 2 * f);
		p = p + 30.82498 * cos(2 * d + m) + 2.6165 * cos(2 * d + m + n);
		p = p + 0.21252 * cos(2 * d + m + 2 * n) - 0.10888 * cos(2 * d + 2 * m - 2 * n);
		p = p + 2.3538 * cos(2 * d + 2 * m - n) + 0.14764 * cos(2 * d + 2 * m);
		p = p + 0.2556 * cos(3 * d - m - n) - 0.15708 * cos(3 * d - m);
		p = p + 0.86243 * cos(3 * d - 2 * n) + 3.25823 * cos(3 * d - n);
		p = p + 0.20099 * cos(3 * d - 2 * f) - 1.41893 * cos(3 * d);
		p = p - 0.21259 * cos(3 * d + m - n) - 0.10766 * cos(3 * d + m);
		p = p - 0.10834 * cos(4 * d - 2 * m - 2 * n) - 0.27906 * cos(4 * d - 2 * m - n);
		p = p - 0.12806 * cos(4 * d - 2 * m) - 1.897 * cos(4 * d - m - 2 * n);
		p = p - 3.95812 * cos(4 * d - m - n) - 1.57145 * cos(4 * d - m);
		p = p - 0.20286 * cos(4 * d - m + n) - 0.51423 * cos(4 * d - 3 * n);
		p = p - 21.63627 * cos(4 * d - 2 * n) - 0.32176 * cos(4 * d - n - 2 * f);
		p = p - 34.78245 * cos(4 * d - n) - 0.50793 * cos(4 * d - 2 * f);
		p = p - 11.64993 * cos(4 * d) - 1.42255 * cos(4 * d + n);
		p = p - 0.13922 * cos(4 * d + 2 * n) + 0.23696 * cos(4 * d + m - 2 * n);
		p = p + 0.5788 * cos(4 * d + m - n) + 0.24453 * cos(4 * d + m);
		p = p - 0.18316 * cos(6 * d - 3 * n) - 0.4225 * cos(6 * d - 2 * n);
		p = p - 0.28663 * cos(6 * d - n);

		distance_terre = p;

	
		p = sin(6378.136 / distance_terre) * r2d * 3600;
		p = floor(p * 1000 + 0.5) / 1000;
		double	p1 = floor(p / 60);

	
		double		diamapparent = atan(3476 / distance_terre) * r2d * 60;
				double	diamapparent1 = floor((diamapparent - floor(diamapparent)) * 60);
		
		asc = r2d / 15 * atan((cos(obliquite) * sin (longitude) -tan(latitude) * sin (obliquite)) / cos(longitude));
		if (asc<0) {asc = asc + 24;}
		if (cos(longitude) < 0) {asc = asc + 12;}
		if (asc > 24) {asc = asc - 24;}

		declin = r2d * asin(sin(latitude) * cos(obliquite) + cos(latitude) * sin (obliquite) * sin(longitude));

		d1 = ((asc-floor(asc)) * 60);
		d2 = ((d1 - floor(d1)) * 60);

		double	ADLune = asc * 15 / r2d;

		d = abs(declin);
		d1 = ((d - floor(d)) * 60);
		d2=  ((d1 - floor(d1)) * 60);

		double DecLune;
		if (declin < 0) {signe = "-"; DecLune = - d / r2d;}
		else {signe = "+"; DecLune = + d / r2d;}

		double ascLune = asc;
		double declinLune = declin;
		Utils.log(asc);
		Utils.log(declin);
	}

}
