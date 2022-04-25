package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.domain.model.Dataset;

public interface IPolicyExecutionPoint {
	void delete(Dataset dataset);
}
