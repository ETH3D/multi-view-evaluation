FROM ubuntu

# Initialize the environment 
RUN apt update
RUN apt install -y libboost-all-dev
RUN apt install libeigen3-dev
RUN DEBIAN_FRONTEND=noninteractive apt-get -yq install libpcl-dev
RUN apt install -y cmake git

# Build the ETH3D binaries
RUN git clone https://github.com/ETH3D/multi-view-evaluation.git
RUN cd multi-view-evaluation; mkdir build; cd build; cmake ../. ; make;
