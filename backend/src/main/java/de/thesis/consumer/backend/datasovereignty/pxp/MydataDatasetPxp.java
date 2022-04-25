package de.thesis.consumer.backend.datasovereignty.pxp;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import lombok.AllArgsConstructor;

import java.util.UUID;

@PxpService(componentName = "dataset-pxp")
@AllArgsConstructor
public class MydataDatasetPxp {

	private final DatasetRepository repository;

	@ActionDescription
	public void deleteDataset(@ActionParameterDescription(name = "datasetId", mandatory = true) final String id) {
		repository.deleteById(UUID.fromString(id));
	}
}
