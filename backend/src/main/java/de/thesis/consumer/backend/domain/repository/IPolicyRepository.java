package de.thesis.consumer.backend.domain.repository;


import de.thesis.consumer.backend.domain.model.Policy;

public interface IPolicyRepository {
	void save(Policy policy);

	Policy findById(String id);
}
