package datasovereignty;

import commands.RemoteLoggingCommand;
import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.AllArgsConstructor;
import usecases.RemoteLoggingUseCase;

import java.util.UUID;

@PxpService(componentName = "remote-local-pxp")
@AllArgsConstructor
public class MydataRemoteLoggingPxp {

	private final ProviderCommunicationPortAdapter providerCommunicationPortAdapter;
	private final RemoteLoggingUseCase remoteLoggingUseCase;

	@ActionDescription(methodName = "log-remote")
	public boolean log(@ActionParameterDescription(name = "url", mandatory = true) final String url,
					   @ActionParameterDescription(name = "id", mandatory = true) final String datasetId
	) {
		remoteLoggingUseCase.handle(new RemoteLoggingCommand(url, UUID.fromString(datasetId)));

		return true;
	}
}
