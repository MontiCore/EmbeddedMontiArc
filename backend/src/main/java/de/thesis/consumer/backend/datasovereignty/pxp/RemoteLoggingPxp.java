package de.thesis.consumer.backend.datasovereignty.pxp;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import de.thesis.consumer.backend.domain.model.Dataset;
import de.thesis.consumer.backend.domain.service.ProviderClient;
import lombok.AllArgsConstructor;

@PxpService(componentName = "logging-remote-pxp")
@AllArgsConstructor
public class RemoteLoggingPxp {

	private final ProviderClient client;

	@ActionDescription(methodName = "log-remote")
	public boolean logDataUsage(@ActionParameterDescription(name = "dataset", mandatory = true) final Dataset dataset) {
		try {
			client.notifyDataOwner(dataset);
		} catch(Exception e){
			return false;
		}

		return true;
	}
}
