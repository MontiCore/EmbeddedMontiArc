package de.thesis.consumer.backend.domain.repository;

import de.thesis.consumer.backend.persistence.entity.TruckData;

import java.util.List;

public interface TruckDataRepository {
	void save(TruckData data);

	List<TruckData> findAll();
}
