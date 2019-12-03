
# Running ETH3D evaluation using Docker

You may want to just run the ETH3d evaluation in Docker instead of installing and running on your local machine. To do so, follow these steps:

1. Make sure docker is installed on your local machine.
2. Run the included script, using the full path to the local directory with the pointclouds you want to evaluate, i.e:

	./runMe.sh /my/local/pointcloudfiles

3. This will put you in a directory (inside the Docker container) where you can run and evaluate your point clouds.
