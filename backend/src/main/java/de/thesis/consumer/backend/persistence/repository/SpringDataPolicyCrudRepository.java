package de.thesis.consumer.backend.persistence.repository;

import de.thesis.consumer.backend.persistence.entity.PolicyEntity;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

import java.util.UUID;

@Repository
public interface SpringDataPolicyCrudRepository extends CrudRepository<PolicyEntity, String> {
}
