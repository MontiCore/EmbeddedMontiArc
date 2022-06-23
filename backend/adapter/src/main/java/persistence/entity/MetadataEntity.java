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

	private String description;

	private double price;

	private String loggingUrl;

	@OneToOne(cascade = CascadeType.ALL)
	private PolicyEntity policy;
}

