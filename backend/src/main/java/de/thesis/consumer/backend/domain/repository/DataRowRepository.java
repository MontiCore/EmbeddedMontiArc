package de.thesis.consumer.backend.domain.repository;

import de.thesis.consumer.backend.domain.model.DataRow;

import java.util.List;

public interface DataRowRepository {
	void save(DataRow row);

	List<DataRow> findAll();
}
