package de.thesis.consumer.backend.domain.repository;


import de.thesis.consumer.backend.domain.model.Policy;

public interface PolicyRepository {
	void save(Policy policy);

	Policy findById(String id);
}
