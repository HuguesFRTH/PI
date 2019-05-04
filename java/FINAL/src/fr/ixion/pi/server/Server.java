package fr.ixion.pi.server;
import java.net.*;
import java.util.Date;

import fr.ixion.pi.ComType;
import fr.ixion.pi.Command;
import fr.ixion.pi.Location;
import fr.ixion.pi.astres.AstreManager;
import fr.ixion.pi.serial.Serial;

import java.io.*;

public class Server
{
    final static int port = 1042;

    public Server()
    {
        try
        {
            ServerSocket socketServeur = new ServerSocket(port);
            System.out.println("Le serveur est à l'écoute...");
            while(true)
            {
                Socket socketClient = socketServeur.accept();
                
                String message = "";
                BufferedReader in = new BufferedReader(new InputStreamReader(socketClient.getInputStream()));
                PrintStream out = new PrintStream(socketClient.getOutputStream());
                if(in.read() != -1) {

                    message = in.readLine();   
                    if(message.contains("[PACKET]")) {
                        message = convertir(message);
                        execute(message);
                    }
                }
              //  message = convertir(message);
                out.println(message);
                socketClient.close();
            }
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }
    
    
    
    public void execute(String mess) throws IOException {
        String[] args = mess.replaceAll(" ", "").split("/");
        int astreID = 444;
        if(args[0].equals(ComType.COM.getLabel())) {
            for(int i = 1; i< args.length;i++) {
                if(args[i].contains("latitude=")) {
                   Location.instance.latitude = Double.parseDouble(args[i].replace("latitude=", ""));
                }
                if(args[i].contains("longitude=")) {
                    Location.instance.longitude = Double.parseDouble(args[i].replace("longitude=", ""));
                 }
                if(args[i].contains("astreID=")) {
                    astreID = Integer.parseInt(args[i].replace("astreID=", ""));
                 }
            }
           Command cmd = AstreManager.instance.returnCMDForID(astreID);
           cmd.talk();
            Serial.instance.sendPacket(cmd);
        }
    }

    
    
    public String convertir(String ent)
    {
        String out = "";
        out = ent;
        out = out.replaceAll("%20", "").replace("HTTP/1.1", "").replace("GET /", "").replace("%C3%A9", "é").replaceAll("ET /", "").replace("[PACKET]","");
        return out;
    }
}