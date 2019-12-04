
# Running ETH3D evaluation using Docker

You may want to just run the ETH3d evaluation in Docker instead of installing and running on your local machine. To do so, follow these steps:

1. Make sure docker is installed on your local machine.
2. Run the included script, using the full path to the local directory with the pointclouds you want to evaluate, i.e:

	./runMe.sh /my/local/pointcloudfiles

3. This will put you in a directory (inside the Docker container) where you can run and evaluate your point clouds.

## Heads up!

Running the ETH3D pointcloud evaluation program can use a lot of memory (depending on the size of the point clouds being compared). Docker has a relatively small default memory setting (2Gb on Mac). You will probably want to increase this before you run comparisons on the ETH3D datasets. From Docker desktop on Mac for example, just open the Docker GUI, go to the *Advanced* tab and increase via the slider:

![alt text][dockerParam]

Setting the memory to >5Gb should be sufficient for most evaluations.

[dockerParam]: https://i.stack.imgur.com/6iWiW.png "Recommend increasing memory to >5Gb"
