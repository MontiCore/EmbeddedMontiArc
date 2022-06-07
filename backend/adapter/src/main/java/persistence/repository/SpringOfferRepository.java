package persistence.repository;

import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;
import persistence.entity.OfferEntity;

import java.util.UUID;

@Repository
public interface SpringOfferRepository extends CrudRepository<OfferEntity, UUID> {
	@Query(value = "SELECT o.* FROM offer o WHERE o.id IN (SELECT DISTINCT d.offer_id FROM dataset d)", nativeQuery = true)
	Iterable<OfferEntity> findAllBoughtOffers();
}
