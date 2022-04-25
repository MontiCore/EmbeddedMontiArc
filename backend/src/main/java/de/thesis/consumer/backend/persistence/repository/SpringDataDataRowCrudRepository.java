package de.thesis.consumer.backend.persistence.repository;

import de.thesis.consumer.backend.persistence.entity.DataRowEntity;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.UUID;

@Repository
public interface SpringDataDataRowCrudRepository extends CrudRepository<DataRowEntity, Long> {
	List<DataRowEntity> findAllByOfferId(UUID offerId);
}
