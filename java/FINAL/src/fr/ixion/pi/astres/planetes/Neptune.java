package fr.ixion.pi.astres.planetes;


import static fr.ixion.pi.Maths.*;

import static fr.ixion.pi.Soleil.*;



public class Neptune extends Planete {

	@Override
	public void constantes() {
		const1 = 84.457994;
		   const2 = 219.885914;
		   const3 = 0.0003205;
		   const4 = 30.11038703542;
		   const5 = 0.00899704;
		   const6 = 0.00000633;
		   const7 = -0.000000002;
		   const8 = 1.779242;
		   const9 = -0.0095436;
		   const10 = -0.0000091;
		   const11 = 37.73063;
		   const12 = 218.4613396;
		   const13 = -0.00007032;
		   const14 = 130.681389;
		   const15 = 1.098935;
		   const16 = 0.00024987;
		   const17 = 68.289;
	}

	@Override
	public int getID() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void update() {
		//--------------- Longitude moyenne (l), demi-grand axe (a), excentricité (e), inclinaison (i), 
		//--------longitude noeud ascendant (m)

	double	l = (const1 + const2 * T + const3 * T2) / r2d;
		l = l - y * floor(l / y);

		double	a = const4;

		double	e = const5 + const6 * T + const7 * T2;
		double	i = (const8 + const9 * T + const10 * T2) / r2d;
		double	m = (const11 + const12 * T + const13 * T2) / r2d;
		m = m - y * floor(m / y);
		double	longitude_noeud = (const14 + const15 * T + const16 * T2) / r2d;

		//--------------- Anomalie moyenne


		//--------Termes périodiques

		double  u = T / 5 + 0.1;
		   double  p = (237.475 + 3034.9061 * T) / r2d;
		   double   q = (265.916 + 1222.1139 * T) / r2d;
		   double   g = (83.76922 + 218.4901 * T) / r2d;
		   double   h = (284.02 + 8.51 * T) / r2d;
		   double   theta = (200.25 - 209.98 * T) / r2d;
		   double   dzeta = (153.71 + 2816.42 * T) / r2d;
		   double   eta = (182.15 + 1003.62 * T) / r2d;

		//--------perturbations

		   double   granda = (-0.589833 + 0.001089 * u) * sin(h) + (-0.056094 + 0.004658 * u) * cos(h) - 0.024286 * sin(2 * h);
		   double  grandb = 438.9 * sin(h) + 426.2 * cos(h) + 112.9 * sin(2 * h) + 108.9 * cos(2 * h);
		   double   grandc = 0.024039 * sin(h) - 0.025303 * cos(h) + 0.006206 * sin(2 * h) - 0.005992 * cos(2 * h);
		   double   grandd = -817 * sin(h) + 8189 * cos(h) + 781 * cos(2 * h);

		//--------corrections

		   l = l + granda / r2d;
		   m = m + granda / r2d - grandc / r2d / e;
		   e = e + grandb / 1000000;
		   a = a + grandd / 1000000;

		//--------équation de Kepler

		   double	grand_e = m;
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

		double	r = a * (1 - e * cos(grand_e));

		   r = r - 0.040596 + 0.004992 * cos(dzeta) + 0.002744 * cos(eta) + 0.002044 * cos(theta) + 0.001051 * cos(2 * theta);

		//--------argument de latitude

		   	u = l + v - m - longitude_noeud;
		u = u - y * floor(u / y);

		if (cos(u) != 0) {d = atan(cos(i) * tan(u));
		   if (cos(u) < 0) {d = d + PI;}
		}
		   else {d = u;}

		//--------longitude écliptique

		l = d + longitude_noeud;

		   l = l + (0.009556 * sin(dzeta) + 0.005178 * sin(eta)) / r2d;

		if (l > 2 * PI) {l = l - 2 * PI;}

		double	b = asin(sin(u) * sin(i));

		   b = b + (0.000336 * cos(2 * theta) * sin(g) + 0.000364 * sin(2 * theta) * cos(g)) / r2d;

		   double	numerateur = r * cos(b) * sin(l - longitude_terre + PI);
		double	denominateur = r * cos(b) * cos(l - longitude_terre + PI) + distance_terre_soleil;

		l = atan(numerateur / denominateur) + longitude_terre + PI;

		if (l > 2 * PI) {l = l - 2 * PI;}
		if (denominateur < 0) {l = l + PI;}

		double	diametre = const17;

		//--------conversion rectangulaire/polaire

		double	xp = r * cos(b) * cos(l) - distance_terre_soleil * cos(latitude_terre) * cos(longitude_terre);
		double	yp = r * cos(b) * sin(l) - distance_terre_soleil * cos(latitude_terre) * sin(longitude_terre);
		double	zp = r * sin(b) - distance_terre_soleil * sin(latitude_terre);

		double	distance_terre = sqrt(numerateur * numerateur + denominateur * denominateur + r * r * sin(b) * sin(b));


		//--------élongation

		omega = 259.18 / r2d - 1934.142 / r2d * T;
		l = l - 0.00479 / r2d * sin(omega);

		double	elongation = r2d * acos(cos(b) * cos(l - longitude_soleil));
		String elongation_planete;
		if ((l < longitude_soleil && longitude_soleil - l < PI) || (l > longitude_soleil && l - longitude_soleil > PI)) 
		{elongation_planete = "Ouest";}
		else {elongation_planete = "Est";}


		double elongationNept = elongation;
		String elongation_planeteNept =elongation_planete;

		String quandNept;
		String commentNept;
		if (elongationNept < 20) {commentNept = "Inobservable"; quandNept = " ";}

		if (elongationNept > 20 && elongationNept < 45){
		  commentNept = "Observable";
		  if (elongation_planeteNept == "Ouest") {quandNept = "en toute fin de nuit";}
		  if (elongation_planeteNept == "Est") {quandNept = "en tout début de soirée";}}

		if (elongationNept > 45 && elongationNept < 120){
		  commentNept = "Observable";
		  if (elongation_planeteNept == "Ouest") {quandNept = "en seconde partie de nuit";}
		  if (elongation_planeteNept == "Est") {quandNept = "en première partie de nuit";}}

		if (elongationNept > 120 && elongationNept < 140) {commentNept = "Observable"; quandNept = "pratiquement toute la nuit";}
		if (elongationNept > 140 && elongationNept < 180) {commentNept = "Observable"; quandNept = "toute la nuit";}

	
		//--------convertion longitude et latitude en ascension droite et déclinaison

		l = l -y * floor(l /y);
		double	l1 = l * r2d;

		d1 = ((l1 - floor(l1)) * 60);
		d2 = ((d1 - floor(d1)) * 60);

	
		double	beta = asin(r * sin(b) / distance_terre);

		double	b1 = abs(beta * r2d);
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

		double	ADNept = asc * 15 / r2d;

		d = abs(declin);
		d1 = ((d - floor(d)) * 60);
		d2=  ((d1 - floor(d1)) * 60);

		double DecNept;
		if (declin < 0) {signe = "-"; DecNept = - d / r2d;}
		else {signe = "+"; DecNept = + d / r2d;}

		double ascNept = asc;
		double declinNept = declin;

		//Magnitude de la planète

		double	dist = R       ;             // rayon vecteur Soleil-Terre
		double	ray = r       ;              // rayon vecteur Soleil-planète
		double	delta = distance_terre ;     // distance Terre-planète

		double	    FV = acos( (ray * ray + delta * delta - dist * dist ) / (2 * ray * delta) );
		    FV = (FV) * r2d;

		    double	    phase = (1 + cos(FV / r2d)) * 50;

		    double	magnitude = - 6.90 + 5 * (log(ray * delta))/ log(10) + 0.001 * FV;
		magnitude = ceil(magnitude * 100)/100	;
	}

}
