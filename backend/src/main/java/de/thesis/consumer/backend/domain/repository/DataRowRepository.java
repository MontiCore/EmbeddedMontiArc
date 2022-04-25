package de.thesis.consumer.backend.domain.repository;

import de.thesis.consumer.backend.domain.model.DataRow;

import java.util.List;
import java.util.UUID;

public interface DataRowRepository {

	List<DataRow> findAllByOfferId(UUID offerId);

	void saveAll(List<DataRow> data);
}
