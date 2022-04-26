package de.thesis.consumer.backend.datasovereignty.pxp;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@PxpService(componentName = "dataset-pxp")
@AllArgsConstructor
@Slf4j
public class MydataDatasetPxp {

	// private final DatasetRepository repository;

	@ActionDescription
	public void deleteDataset(@ActionParameterDescription(name = "datasetId", mandatory = true) final String id) {
		log.error("executing Policy execution point: " + id);
		System.err.println("hier bin ich");
		// repository.deleteById(UUID.fromString(id));
	}
}
