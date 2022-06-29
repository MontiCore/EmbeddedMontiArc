package persistence.entity;


import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;

@Entity
@Table(name = "metadata")
@Getter
@Setter
public class MetadataEntity {
	@Id
	@GeneratedValue(strategy = GenerationType.SEQUENCE)
	private int id;

	@Column(columnDefinition = "TEXT")
	private String title;

	private String provider;

	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	private String loggingUrl;

	@OneToOne(cascade = CascadeType.ALL, orphanRemoval = true)
	private PolicyEntity policy;
}

