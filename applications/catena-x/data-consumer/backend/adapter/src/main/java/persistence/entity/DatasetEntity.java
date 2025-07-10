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

	private UUID offerId;

	@OneToOne(cascade = CascadeType.ALL, orphanRemoval = true)
	private MetadataEntity metadata;

	@OneToMany(mappedBy = "dataset", fetch = FetchType.EAGER, cascade = CascadeType.ALL, orphanRemoval = true)
	private List<DataRowEntity> data;

	private LocalDateTime boughtAt;
}
