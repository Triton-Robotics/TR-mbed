# Use a base image with build tools like CMake and GCC
FROM ubuntu:22.04

# Set the environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
RUN ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -y tzdata

# Install necessary packages
RUN apt-get update && apt-get install -y \
	build-essential                 \
	git				\
 	cmake                           \
  	python3-dev                     \
  	python3-pip                     \
  	gcc-arm-none-eabi               \
  	libnewlib-arm-none-eabi         \
  	libstdc++-arm-none-eabi-newlib  \
  	openocd                         \
  	ninja-build			\
  	python3				\
  	python3-pip			\
    && rm -rf /var/lib/apt/lists/*
    
# Install the required Python pip packages
RUN pip3 install    \
  	mbed-tools  \
	prettytable \
  	future      \
  	jinja2      \
  	intelhex

# Set the working directory inside the container
WORKDIR /app

# Create a build directory for CMake but do not copy the source code yet (this allows caching)
RUN mkdir -p /app/build
WORKDIR /app/build

# Default command that can be overridden
CMD ["/bin/bash"]
