# Use a imagem base do ROS
FROM osrf/ros:humble-desktop

# Crie um diretório de trabalho
WORKDIR /ros_ws

# Fonte do ambiente ROS
RUN apt-get update && apt-get install -y \
      x11-apps \
      && rm -rf /usr/share/doc/* && \
      rm -rf /usr/share/info/* && \
      rm -rf /tmp/* && \
      rm -rf /var/tmp/*
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
