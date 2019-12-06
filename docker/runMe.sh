docker build -t="eth3d-ubuntu" .;
docker run -w /multi-view-evaluation/build -v $1:/multi-view-evaluation/build/pointclouds2test -it eth3d-ubuntu;
