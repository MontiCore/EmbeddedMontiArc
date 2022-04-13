package de.thesis.consumer.backend.persistence;

import de.thesis.consumer.backend.entities.TruckData;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface SpringDataTruckDataCrudRepository extends CrudRepository<TruckData, Long> {
	List<TruckData> findAll();
}
