#!/bin/bash
docker run --cpus="20" -m 16384m -v $(pwd):/root/mnist --entrypoint /bin/bash -it 1b6cd7cd497b
