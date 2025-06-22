FROM ubuntu:24.04

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    v4l-utils

# Download https://github.com/prefix-dev/pixi/releases/download/v0.48.2/pixi-aarch64-unknown-linux-musl.tar.gz and extract to /usr/local/bin
RUN wget https://github.com/prefix-dev/pixi/releases/download/v0.48.2/pixi-$(uname -i)-unknown-linux-musl.tar.gz -O pixi.tar.gz && \
    tar -xzf pixi.tar.gz -C /usr/local/bin && \
    rm pixi.tar.gz

# Set up the workspace
WORKDIR /workspace

# Set the default command
CMD ["bash"]
