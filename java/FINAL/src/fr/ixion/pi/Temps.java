/**
 *  Ixion
 */
package fr.ixion.pi;

import java.text.SimpleDateFormat;
import java.util.TimeZone;
import static fr.ixion.pi.Maths.*;

/**
 * @author Ixion
 */
public class Temps
{
    public Date date;
    public static Temps instance;
    public double T;
    public double T2;
    public double T3;
    public double TS;
    // TEMPS A L'AN 2000 EN 2019
    public double T_2000 = 0.19307511826627807;

    public Temps()
    {
        instance = this;
        firstDate();
    }
    public void firstDate()
    {
        SimpleDateFormat txtDate = new SimpleDateFormat("ss/mm/HH/dd/MM/yyyy");
        txtDate.setTimeZone(TimeZone.getTimeZone("UTC"));
        Utils.log(txtDate.format(new java.util.Date()));
        String[] strs = txtDate.format(new java.util.Date()).split("/");
        int Y = Integer.valueOf(strs[5]);
        int M = Integer.valueOf(strs[4]);
        double D = dayToDay(Integer.valueOf(strs[3]), Integer.valueOf(strs[2]), Integer.valueOf(strs[1]), Integer.valueOf(strs[0]));
        date = new Date(Y, M, D, true);
        date.addMore(Integer.valueOf(strs[2]), Integer.valueOf(strs[1]), Integer.valueOf(strs[0]));
        Utils.log(date.jj + "");
        T = (double)(date.jj - 2415020.0) / 36525;
        T2 = pow(T, 2);
        T3 = pow(T, 3);

        double Jts = floor(date.jj + 0.5) - 0.5; // jour julien à 0h00 UT
        double Tts = (Jts - 2415020.0) / 36525;
        double ts0 = 0.276919398 + 100.0021359 * Tts + 0.000001075 * Tts * Tts;
        TS = (ts0 - floor(ts0)) * 24; // temps sidéral à Greenwich à 0h00 UT

        double deltaT_2019 = 69.22;
        double coeff = 86400;
        double dtAm = +102 + (102 * T3) + (25.3 * T3 * T3) + 0.56 * (Y - 2100);
        double jj2 = Jts + (dtAm / coeff);

        T_2000 = (jj2 - 2451545) / 36525;
        Utils.log(T_2000);
    }
    
    
    public void updateDate()
    {
        SimpleDateFormat txtDate = new SimpleDateFormat("ss/mm/HH/dd/MM/yyyy");
        txtDate.setTimeZone(TimeZone.getTimeZone("UTC"));
        //Utils.log(txtDate.format(new java.util.Date()));
        String[] strs = txtDate.format(new java.util.Date()).split("/");
        int Y = Integer.valueOf(strs[5]);
        int M = Integer.valueOf(strs[4]);
        double D = dayToDay(Integer.valueOf(strs[3]), Integer.valueOf(strs[2]), Integer.valueOf(strs[1]), Integer.valueOf(strs[0]));
        date = new Date(Y, M, D, true);
        date.addMore(Integer.valueOf(strs[2]), Integer.valueOf(strs[1]), Integer.valueOf(strs[0]));
      //  Utils.log(date.jj + "");
        T = (double)(date.jj - 2415020.0) / 36525;
        T2 = pow(T, 2);
        T3 = pow(T, 3);

        double Jts = floor(date.jj + 0.5) - 0.5; // jour julien à 0h00 UT
        double Tts = (Jts - 2415020.0) / 36525;
        double ts0 = 0.276919398 + 100.0021359 * Tts + 0.000001075 * Tts * Tts;
        TS = (ts0 - floor(ts0)) * 24; // temps sidéral à Greenwich à 0h00 UT

        double deltaT_2019 = 69.22;
        double coeff = 86400;
        double dtAm = +102 + (102 * T3) + (25.3 * T3 * T3) + 0.56 * (Y - 2100);
        double jj2 = Jts + (dtAm / coeff);

        T_2000 = (jj2 - 2451545) / 36525;
      //  Utils.log(T_2000);
    }

    public static double dayToDay(int day, int hours, int mins, int secs)
    {
        double h = (double)hours / 24;
        double m = (double)mins / 1440;
        double s = (double)secs / 86400;
        double d = day;
        double realDay = d + h + m + s;
        return realDay;
    }
}
