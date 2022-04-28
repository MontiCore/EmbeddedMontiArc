package de.thesis.consumer.backend.datasovereignty.pxp;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.thesis.consumer.backend.domain.model.Dataset;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import lombok.AllArgsConstructor;

import java.time.LocalDate;

@PxpService(componentName = "expiration-check-pxp")
@AllArgsConstructor
public class ExpirationCheckPxp {

	private final DatasetRepository repository;

	@ActionDescription(methodName = "check-expiration")
	public boolean deleteDataset() {
		for (Dataset dataset : repository.findAll()) {
			if (dataset.getExpiresOn() != null && dataset.getExpiresOn().isBefore(LocalDate.now())) {
				repository.deleteById(dataset.getId());
			}
		}

		return true;
	}
}
