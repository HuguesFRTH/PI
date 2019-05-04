/**
 *  Ixion
 */
package fr.ixion.pi;

import java.io.IOException;

import fr.ixion.pi.astres.planetes.*;
import fr.ixion.pi.serial.Serial;
import fr.ixion.pi.server.Server;

/**
 * @author Ixion
 */
public class Main
{

    public static void main(String[] args) throws IOException, InterruptedException
    {
        new Temps();

        // new Serial();
    //   new Main();
    }

    public Main() throws IOException, InterruptedException
    {
        new Serial();

        new Temps();
        new Location();

        new Soleil();

        // new Server();
        // new All();
    }
}
