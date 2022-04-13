package de.thesis.consumer.backend.persistence;

import de.thesis.consumer.backend.domain.TruckDataRepository;
import de.thesis.consumer.backend.entities.TruckData;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
@AllArgsConstructor
public class TruckDataRepositoryImpl implements TruckDataRepository {

	private final SpringDataTruckDataCrudRepository repository;

	@Override
	public void save(TruckData data) {
		repository.save(data);
	}

	@Override
	public List<TruckData> findAll() {
		return repository.findAll();
	}
}
