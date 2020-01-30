# In order to support rendering 
FROM nvidia/opengl:1.0-glvnd-runtime

WORKDIR /
RUN apt-get update
RUN apt-get install python3.5 python3-pip python3-tk git vim -y
RUN pip3 install --upgrade pip
RUN pip3 install pybullet numpy matplotlib

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.5 1

RUN git clone https://github.com/kopernikusai/shapes3d.git /shapes3d

WORKDIR /shapes3d
RUN pip3 install -e .

# To run on gpu and with opengl
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

COPY docker/entrypoint.sh /
COPY docker/empty_entrypoint.sh /
ENTRYPOINT ["/bin/sh", "/entrypoint.sh"]
