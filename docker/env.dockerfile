# In order to support rendering 
FROM nvidia/opengl:1.0-glvnd-runtime

WORKDIR /
RUN apt-get update
RUN apt-get install python3.5 python3-pip git vim -y
RUN pip3 install --upgrade pip
RUN pip3 install pybullet numpy matplotlib

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.5 1

RUN git clone https://github.com/kopernikusai/shapes3d /shapes3d

WORKDIR /shapes3d
RUN pip3 install -e .
