package persistence.repository;

import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;
import persistence.entity.OfferEntity;

import java.util.UUID;

@Repository
public interface SpringOfferRepository extends CrudRepository<OfferEntity, UUID> {
}
