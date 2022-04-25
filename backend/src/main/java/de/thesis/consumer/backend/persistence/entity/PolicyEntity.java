package de.thesis.consumer.backend.persistence.entity;

import com.fasterxml.jackson.annotation.JsonIgnore;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import javax.persistence.*;

@Entity
@Table(name = "policy")
@Getter
@Setter
public class PolicyEntity {
	// TODO die brauche ich nicht
	@Id
	private String id;
	@Column(length = 2048)
	private String rawValue;

	@OneToOne(mappedBy = "policy")
	@JsonIgnore
	private OfferEntity offer;

	@OneToOne(mappedBy = "policy")
	@JsonIgnore
	private DatasetEntity dataset;
}
