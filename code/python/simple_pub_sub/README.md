# Simple Publisher Subscriber

Código criado a partir [deste](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/basics/ROS2-Simple-Publisher-Subscriber.html) tutorial.

## Compilar o código

```bash
$ colcon build --symlink-install
```

## Configurar terminal

```bash
$ source install/local_setup.bash
```

## Executar talker
```bash
$ ros2 run simple_pub_sub talker
```

## Executar listener

Executar em um terminal distinto.

```bash
$ ros2 run simple_pub_sub listener
```