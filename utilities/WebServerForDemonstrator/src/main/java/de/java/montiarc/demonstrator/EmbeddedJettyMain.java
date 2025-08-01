/* (c) https://github.com/MontiCore/monticore */
package de.java.montiarc.demonstrator;

import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.servlet.ServletContextHandler;
import de.java.montiarc.demonstrator.servlet.MainServlet;

public class EmbeddedJettyMain {

    public static void main(String[] args) throws Exception {

        if(args.length == 0)
        {
            System.out.println("Usage example: java -jar web-server-demonstartor.jar port");
            System.exit(0);
        }
        else {

            Server server = new Server(Integer.parseInt(args[0]));
            ServletContextHandler handler = new ServletContextHandler(server, "/receiver");
            handler.addServlet(MainServlet.class, "/");
            server.start();
        }
    }
}
