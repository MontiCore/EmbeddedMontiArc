package de.thesis.consumer.backend.persistence.entity;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.util.UUID;

@Getter
@Setter
@Entity
@Table(name = "offer")
public class OfferEntity {

	@Id
	private UUID id;

	private String title;

	private String provider;

	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	@OneToOne(cascade = CascadeType.ALL)
	@JoinColumn(name = "policy_id", referencedColumnName = "id")
	private PolicyEntity policy;
}
