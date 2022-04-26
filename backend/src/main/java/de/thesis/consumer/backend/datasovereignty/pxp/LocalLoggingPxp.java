package de.thesis.consumer.backend.datasovereignty.pxp;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;

import java.time.LocalDateTime;

@PxpService(componentName = "logging-local-pxp")
@Slf4j
public class LocalLoggingPxp {

	@ActionDescription(methodName = "log-local")
	public boolean deleteDataset(@ActionParameterDescription(name = "datasetId", mandatory = true) final String datasetId) {
		log.info("Dataset {} accessed at {}", datasetId, LocalDateTime.now());

		return true;
	}
}
