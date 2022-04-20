package de.thesis.consumer.backend.persistence.repository;

import de.thesis.consumer.backend.domain.repository.TruckDataRepository;
import de.thesis.consumer.backend.persistence.entity.TruckData;
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
