package de.thesis.consumer.backend.datasovereignty.pep;

import de.fraunhofer.iese.mydata.policy.event.Event;
import de.thesis.consumer.backend.domain.InhibitionException;
import de.thesis.consumer.backend.domain.PolicyEnforcementPoint;
import de.thesis.consumer.backend.domain.model.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

@Component
@AllArgsConstructor
public class DatasetPolicyEnforcementPoint implements PolicyEnforcementPoint<Dataset> {

	private final MYDATADatasetPEP pep;

	@Override
	public Dataset enforce(Dataset dataset) throws InhibitionException {
		Event event = pep.enforceDataset(dataset).toBlocking().first();
		return event.getParameterValue("dataset", Dataset.class);
	}
}
