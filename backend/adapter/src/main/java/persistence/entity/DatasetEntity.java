package persistence.entity;


import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;

@Entity
@Table(name = "dataset")
@Getter
@Setter
public class DatasetEntity {
	@Id
	private UUID id;

	@OneToOne
	private OfferEntity offer;

	@OneToOne(cascade = CascadeType.ALL)
	private MetadataEntity metadata;

	@OneToMany(mappedBy = "dataset", cascade = CascadeType.ALL, orphanRemoval = true)
	private List<DataRowEntity> data;

	private LocalDateTime boughtAt;
}
