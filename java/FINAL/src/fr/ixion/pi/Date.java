package fr.ixion.pi;

public class Date {

	public int y;
	public int m;
	public double d;
	public static Date grego = new Date(1582, 10, 15,false);
	public int Y;
	public int M;
	public double jj;
	private double B = 0;
	private boolean isGrego;
	public int minutes;
	public int heures;
	public int secondes;

	public Date(int y, int m, double d, boolean load) {
		this.y = y;
		this.m = m;
		this.d = d;
		this.M = m;
		this.Y = y;
		if (load) {
			load();
		}
	}

	public boolean isGrego() {
		return isGrego;
	}

	public void load() {
		isGrego = isGrego(this);
		if (m <= 2) {
			M = m + 12;
			Y = y - 1;
		}
		// Calcul of Julien Day
		if (!isGrego()) {
			B = 0;
		} else {
			double A = Math.floor(Y / 100);
			B = 2 - A + Math.floor(A / 4);
		}
		double x = Math.floor(365.25 * (Y + 4716));
		double yy =  Math.floor(30.6001 * (M + 1));
		jj = x + yy + d + B - 1524.5;

	}

	/*
	 */
	public static boolean isGrego(Date dt) {

		if (dt.y < grego.y) {
			return false;
		}
		if (dt.y > grego.y) {
			return true;
		}
		if (dt.y == grego.y) {
			if (dt.m < grego.m) {
				return false;
			}
			if (dt.m > grego.m) {
				return true;
			}
			if (dt.m == grego.m) {
				if (dt.d < grego.d) {
					return false;
				} else {
					return true;
				}
			}
		}
		return true;
	}

    /**
     * @param valueOf
     * @param valueOf2
     * @param valueOf3
     */
    public void addMore(Integer valueOf, Integer valueOf2, Integer valueOf3)
    {
        heures = valueOf;
       minutes = valueOf2;
       secondes = valueOf3;
        
    }
}
