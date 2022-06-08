package datasovereignty;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.AllArgsConstructor;

import java.util.UUID;

@PxpService(componentName = "remote-local-pxp")
@AllArgsConstructor
public class RemoteLoggingExecutionPortAdapter {

	private final ProviderCommunicationPortAdapter providerCommunicationPortAdapter;

	@ActionDescription(methodName = "log-remote")
	public boolean log(@ActionParameterDescription(name = "url", mandatory = true) final String url,
					   @ActionParameterDescription(name = "id", mandatory = true) final String datasetId
	) {
		providerCommunicationPortAdapter.notifyProvider(url, UUID.fromString(datasetId));

		return true;
	}
}
