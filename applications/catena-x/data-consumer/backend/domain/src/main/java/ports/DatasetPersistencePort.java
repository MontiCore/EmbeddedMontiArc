package ports;


import entity.Dataset;

import java.util.UUID;

public interface DatasetPersistencePort {

	void save(Dataset dataset);

	Iterable<Dataset> findAll();

	Dataset findById(UUID id);

	void deleteById(UUID id);

	void deleteByOfferId(UUID offerId);
}
