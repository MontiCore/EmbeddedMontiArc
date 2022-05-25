package ports;

import entity.DataRow;

import java.util.UUID;

public interface DataPersistencePort {

	Iterable<DataRow> findAllByOfferId(UUID offerId);

	void saveAll(Iterable<DataRow> data);
}
