package datasovereignty;

import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;
import ports.RemoteLoggingExecutionPort;

@Component
@AllArgsConstructor
public class RemoteLoggingExecutionPortAdapter implements RemoteLoggingExecutionPort {

	private final ProviderCommunicationPortAdapter providerCommunicationPortAdapter;

	@Override
	@ActionDescription(methodName = "log-remote")
	public boolean log(@ActionParameterDescription(name = "url", mandatory = true) final String url,
					   @ActionParameterDescription(name = "id", mandatory = true) final String datasetId
	) {
		providerCommunicationPortAdapter.notifyProvider(url, datasetId);

		return true;
	}

	@Override
	public String getName() {
		return "logging-remote-pxp";
	}
}
