package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.TruckData;

import java.util.List;

public interface TruckDataRepository {
	void save(TruckData data);

	List<TruckData> findAll();
}
