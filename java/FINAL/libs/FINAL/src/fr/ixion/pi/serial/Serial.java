/**
 *  Ixion
 */
package fr.ixion.pi.serial;

import java.io.IOException;

import com.fazecast.jSerialComm.SerialPort;

import fr.ixion.pi.Command;
import fr.ixion.pi.packet.Packet;

/**
 * @author Ixion
 */
public class Serial
{

    public static Serial instance;
    public String PORT = "COM5";
    public int baud = 115200;
    private SerialPort sp;

    public Serial() throws IOException, InterruptedException
    {
        open();
//        for(Integer i = 0; i < 5; ++i)
//        {
//            sp.getOutputStream().write(i.byteValue());
//            sp.getOutputStream().flush();
//            System.out.println("Sent number: " + i);
//            Thread.sleep(1000);
//        }
    }

    
    public void sendPacket(Command cm) throws IOException
    {
        String message = cm.astreID + "/" + cm.hauteur +"/" + cm.azimut;
        sp.getOutputStream().write(message.getBytes());
        sp.getOutputStream().flush();
    }
    
    public void open() {
        sp = SerialPort.getCommPort(PORT); // device name TODO: must be changed
        sp.setComPortParameters(baud, 8, 1, 0); // default connection settings for Arduino
        sp.setComPortTimeouts(SerialPort.TIMEOUT_WRITE_BLOCKING, 0, 0); // block until bytes can be written

        if(sp.openPort())
        {
            System.out.println("Port is open :)");
        }
        else
        {
            System.out.println("Failed to open port :(");
            return;
        }
    }
    
    public void close() {
        if(sp.closePort())
        {
            System.out.println("Port is closed :)");
        }
        else
        {
            System.out.println("Failed to close port :(");
            return;
        }
    }

}
