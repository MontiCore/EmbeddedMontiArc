dyna:
	./toolchain/files/dyna.sh
	@echo "FEM-simulation finished."
	
clean:
	@echo "Cleaning up..."
	cd toolchain/files/Lattice_Structures/ && ./clean.sh
	cd toolchain/preprocessing/h5/raw/ && ./clean.sh

preprocessing:
	python toolchain/preprocessing/copy_preprocessing.py
	@echo "Preprocessing..."
	./toolchain/preprocessing/preprocessing.sh
