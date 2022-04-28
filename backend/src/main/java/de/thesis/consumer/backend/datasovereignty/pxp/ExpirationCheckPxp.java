package de.thesis.consumer.backend.datasovereignty.pxp;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import lombok.AllArgsConstructor;

import java.time.LocalDateTime;
import java.util.UUID;

@PxpService(componentName = "expiration-check-pxp")
@AllArgsConstructor
public class ExpirationCheckPxp {

	@ActionDescription(methodName = "check-expiration")
	public boolean deleteDataset() {
		System.err.println("I was executed at " + LocalDateTime.now());
		return true;
	}
}
