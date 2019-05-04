package fr.ixion.pi;

public class Maths {

    public static double PI = Math.PI;
	public static double d2r = PI / 180;
	public static double r2d = 180 / PI;

    public static double y = 2* PI;
	

	
	public static double sin(double nb) {
		return Math.sin((nb));
	}

	public static double cos(double nb) {
		return Math.cos((nb));
	}
	
	public static double tan(double nb) {
		return Math.tan((nb));
	}
	
	public static double acos(double nb) {
		return Math.acos((nb));
	}
	
	public static double asin(double nb) {
		return Math.asin((nb));
	}
	
	public static double atan(double nb) {
		return Math.atan((nb));
	}
	

	public static double pow(double nb, int power) {
		return Math.pow(nb, power);
	}

	public static double sqrt(double nb) {
		return Math.sqrt(nb);
	}

	public static double abs(double nb) {
		return Math.abs(nb);
	}

	public static double floor(double nb) {
		return Math.floor(nb);
	}

	public static double tourValue(double nb) {
		double n = nb - ((floor(nb / 360)) * 360);
		return n;
	}

	public static double toDegrees(double nb) {
		// return nb;
		return Math.toDegrees(nb);
	}

	public static double toRadians(double nb) {
		// return nb;
		return Math.toRadians(nb);
	}

	public static double angDMS(double D, double M, double S) {
		double DMS = D;
		DMS += (M * 1) / 60;

		DMS += (S * 1) / 3600;
		System.out.println(DMS);
		return DMS;
	}

	
	
	public static String angD(double D) {
		String str = (int)floor(D)+"°" + (int)floor((D - floor(D)) * 60)+ "'" +((((D - floor(D))*60) - ((int)floor((D - floor(D)) * 60))) * 60)+ "''";
		Utils.log(str);
		return str;
	}

	public static double log(double d) {
        return Math.log(d);
    }
	public static double ceil(double d) {
        return Math.ceil(d);
    }
}
