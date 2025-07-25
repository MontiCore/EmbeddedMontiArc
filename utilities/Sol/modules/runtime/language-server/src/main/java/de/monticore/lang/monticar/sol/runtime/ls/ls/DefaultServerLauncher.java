/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import com.google.common.flogger.FluentLogger;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.options.OptionsService;
import de.se_rwth.commons.logging.Log;
import org.eclipse.lsp4j.jsonrpc.Launcher;
import org.eclipse.lsp4j.launch.LSPLauncher;
import org.eclipse.lsp4j.services.LanguageClient;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.Set;

@Singleton
public class DefaultServerLauncher extends AbstractServerLauncher {
    protected final FluentLogger logger;
    protected final DuplexLanguageServer server;
    protected final OptionsService options;

    @Inject
    protected DefaultServerLauncher(Set<ServerLauncherContribution> contributions, DuplexLanguageServer server,
                                    OptionsService options) {
        super(contributions);

        this.logger = FluentLogger.forEnclosingClass();
        this.server = server;
        this.options = options;
    }

    @Override
    public void launch(String[] arguments) throws Exception {
        this.initialize(arguments);
        this.configure(arguments);

        Log.enableFailQuick(false);

        if (this.options.hasOption(PortOption.OPTION)) this.launchViaSocket();
        else this.launchViaStream();
    }

    protected void launchViaStream() {
        Launcher<LanguageClient> launcher = LSPLauncher.createServerLauncher(this.server, System.in, System.out);

        this.server.setRemoteProxy(launcher.getRemoteProxy());
        launcher.startListening();
        this.logger.atInfo().log("Launching application with connection via stream...");
    }

    protected void launchViaSocket() throws Exception {
        int port = this.options.getOptionAsInteger(PortOption.OPTION);
        Socket socket = new Socket("localhost", port);
        InputStream in = socket.getInputStream();
        OutputStream out = socket.getOutputStream();
        Launcher<LanguageClient> launcher = LSPLauncher.createServerLauncher(this.server, in, out);

        this.server.setRemoteProxy(launcher.getRemoteProxy());
        launcher.startListening();
        this.logger.atInfo().log("Launching application with connection on port %d...", port);
    }
}
