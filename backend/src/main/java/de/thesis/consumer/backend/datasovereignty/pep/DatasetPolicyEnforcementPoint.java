package de.thesis.consumer.backend.datasovereignty.pep;

import de.fraunhofer.iese.mydata.policy.event.Event;
import de.thesis.consumer.backend.domain.IPolicyEnforcementPoint;
import de.thesis.consumer.backend.domain.model.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

@Component
@AllArgsConstructor
public class DatasetPolicyEnforcementPoint implements IPolicyEnforcementPoint<Dataset> {

	private final MydataDatasetPep pep;

	@Override
	public Dataset enforce(Dataset dataset) {
		Event event = pep.enforce(dataset, dataset.getId().toString()).toBlocking().first();
		return event.getParameterValue("dataset", Dataset.class);
	}
}
