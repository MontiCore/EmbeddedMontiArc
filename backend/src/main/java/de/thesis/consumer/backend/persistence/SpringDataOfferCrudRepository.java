package de.thesis.consumer.backend.persistence;

import de.thesis.consumer.backend.entities.Offer;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface SpringDataOfferCrudRepository extends CrudRepository<Offer, Long> {
	List<Offer> findAll();
}
