package de.thesis.consumer.backend.datasovereignty.pxp;

import de.thesis.consumer.backend.domain.IPolicyExecutionPoint;
import de.thesis.consumer.backend.domain.model.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

@Component
@AllArgsConstructor
public class PolicyExecutionPointImpl implements IPolicyExecutionPoint {

	private final MydataDatasetPxp pxp;

	@Override
	public void delete(Dataset dataset) {
		pxp.deleteDataset(dataset.getId().toString());
	}
}
