package de.java.montiarc.demonstrator;

import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.servlet.ServletContextHandler;
import de.java.montiarc.demonstrator.servlet.ExampleServlet;

public class EmbeddedJettyMain {

    public static void main(String[] args) throws Exception {

        Server server = new Server(7070);
        ServletContextHandler handler = new ServletContextHandler(server, "/receiver");
        handler.addServlet(ExampleServlet.class, "/");
        server.start();

    }
}



