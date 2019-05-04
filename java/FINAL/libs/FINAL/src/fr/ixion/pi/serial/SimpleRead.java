/**
 *  Ixion
 */
package fr.ixion.pi.serial;

import java.io.*;
import java.util.*;
import javax.comm.*;

/**
 * Class declaration
 *
 *
 * @author
 * @version 1.8, 08/03/00
 */
public class SimpleRead {public static void main(String[] args) {
    System.out.println("Program Started!!!");

    CommPortIdentifier serialPortId;

    Enumeration enumComm;

    enumComm = CommPortIdentifier.getPortIdentifiers();

    while(enumComm.hasMoreElements())
    {
    serialPortId = (CommPortIdentifier)enumComm.nextElement();
    if(serialPortId.getPortType() == CommPortIdentifier.PORT_PARALLEL)
    {
    System.out.println(serialPortId.getName());
    }
    }

    System.out.println("Program Finished Sucessfully ");
    }

    }