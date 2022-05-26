package persistence.entity;


import com.fasterxml.jackson.annotation.JsonIgnore;
import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;

@Entity
@Table(name = "policy")
@Getter
@Setter
public class PolicyEntity {
	@Id
	private String id;

	@Column(columnDefinition = "TEXT")
	private String rawValue;

	@OneToOne(mappedBy = "policy")
	@JsonIgnore
	private OfferEntity offer;

	@OneToOne(mappedBy = "policy")
	@JsonIgnore
	private OfferEntity dataset;
}

