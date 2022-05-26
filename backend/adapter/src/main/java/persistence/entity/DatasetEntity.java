package persistence.entity;


import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;

@Entity
@Table(name = "policy")
@Getter
@Setter
public class DatasetEntity {
	@Id
	private UUID id;

	private String title;

	private String provider;

	@Column(columnDefinition = "TEXT")
	private String description;

	private double price;

	private LocalDateTime boughtAt;

	private LocalDate expiresOn;

	@OneToOne(cascade = CascadeType.ALL)
	@JoinColumn(referencedColumnName = "id")
	private PolicyEntity policy;

	@OneToMany(mappedBy = "dataset", cascade = CascadeType.ALL)
	private List<DataRowEntity> data;
}

