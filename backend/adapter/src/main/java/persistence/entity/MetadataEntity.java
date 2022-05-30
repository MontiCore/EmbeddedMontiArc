package persistence.entity;


import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;

@Entity
@Table(name = "metadata")
@Getter
@Setter
public class MetadataEntity {
	String title;
	String provider;
	@Id
	@GeneratedValue
	private Long id;
	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	@OneToOne(cascade = CascadeType.ALL)
	private PolicyEntity policy;
}

