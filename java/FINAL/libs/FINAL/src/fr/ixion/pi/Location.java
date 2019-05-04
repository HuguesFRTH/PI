/**
 *  Ixion
 */
package fr.ixion.pi;

/**
 * @author Ixion
 */
public class Location
{
    public static Location instance;
    public double latitude;
    public double longitude;
    
    public Location()
    {
        latitude = 45;
        longitude = 0;
        instance = this;
    }
    
    public void setLatitude(double latitude) {
     this.latitude += latitude;   
    }
    
    public void setLongitude(double longitude) {
        this.longitude += longitude;   
       }
}
