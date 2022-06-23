package persistence.repository;

import org.springframework.data.jpa.repository.Modifying;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.CrudRepository;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;
import persistence.entity.OfferEntity;

import javax.transaction.Transactional;
import java.util.UUID;

@Repository
public interface SpringOfferRepository extends CrudRepository<OfferEntity, UUID> {
	@Query(value = "SELECT o.* FROM offer o WHERE o.id IN (SELECT DISTINCT d.offer_id FROM dataset d)", nativeQuery = true)
	Iterable<OfferEntity> findAllBoughtOffers();

	@Transactional
	@Modifying
	@Query(value =
			"UPDATE data_row SET offer = null WHERE offer = :offerId ; " +
					"DELETE FROM offer WHERE id = :offerId",
			nativeQuery = true)
	void deleteById(@Param("offerId") UUID offerId);

	@Transactional
	@Modifying
	@Query(value =
			"DELETE FROM data_row WHERE dataset = (SELECT id FROM dataset WHERE offer_id = :offerId );" +
					"DELETE FROM dataset WHERE offer_id = :offerId ; " +
					"DELETE FROM offer WHERE id = :offerId",
			nativeQuery = true)
	void deleteDatasetDataRowOfferById(@Param("offerId") UUID offerId);
}
