package de.thesis.consumer.backend.persistence.repository;

import de.thesis.consumer.backend.persistence.entity.TruckData;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface SpringDataTruckDataCrudRepository extends CrudRepository<TruckData, Long> {
	List<TruckData> findAll();
}
