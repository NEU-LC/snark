# sample dockerfile for tensor-gpu

FROM tensorflow/tensorflow:latest-gpu
 
RUN apt-get update \
        && apt-get install -y --no-install-recommends git python-pip python-dev \
        && apt-get clean \
        && rm -rf /var/lib/apt/lists/*
 
RUN pip install --upgrade setuptools pip
RUN pip --no-cache-dir install numpy tensorflow-gpu \
        && python -m ipykernel.kernelspec
 
#RUN useradd -d /home/dummy -ms /bin/bash -g root -G sudo -p dummy dummy
#USER dummy
#WORKDIR /home/dummy
 
# quick and dirty: just copy tensor-cat, since for now, it does not depend on any comma or snark stuff
RUN mkdir -p ~/src \
        && cd ~/src \
        && git clone https://github.com/acfr/snark.git \
        && cp ~/src/snark/machine_learning/deep_learning/tensorflow/applications/tensor-cat /usr/local/bin
