dyna:
	./toolchain/files/dyna.sh
	@echo "FEM-simulation finished."

clean:
	@echo "Cleaning up..."
	cd toolchain/preprocessing/h5/raw/ && ./clean.sh

preprocessing:
	python toolchain/preprocessing/copy_preprocessing.py
	@echo "Preprocessing..."
	./toolchain/preprocessing/preprocessing.sh
