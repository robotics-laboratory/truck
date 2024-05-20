## ICP Odometry 
## Инструкция по построению карты

Докер контейнер с всеми нужными библиотеками собран под amd64

```bash
git clone -b init_g2o_for_icp_odom https://github.com/robotics-laboratory/truck.git
```

Далее качаем file_1_atrium с яндекс диска https://disk.yandex.ru/d/yGQ-kgzOadYZGA/file_1_atrium.db3
И добавляем его в директорию по пути <путь до репозитория>/tuck/packages/icp_odometry/bags/file_1_atrium.db3 (Директорию bags необходимо создать)

```bash
cd truck
docker compose up -d
docker exec -it truck-example bash
cd truck/packages
colcon build --merge-install --packages-up-to icp_odometry
ros2 run icp_odometry icp_odometry_node
```

Далее результат вы увидете в директории <путь до репозитория>/tuck/packages/icp_odometry/map_result

Внутри будет mcap файл, который можно открыть через сайт http://foxglove.robotics-lab.ru/ (чтобы точно все работало, лучше через браузер google chrome).
Чтобы открыть нужно выбрать вариант "Open local file" и далее выбрать mcap файл. 

Чтобы перезапустить построение карты нужно либо поменять путь до результата result_path в icp_odometry/config/map_builder.yaml или удалить старую папку map_result

Кроме этого, в том же конфиге можно покрутить параметры
