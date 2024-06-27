Пример работы с библиотекой:

```c++
TEST() {
  // --- Инициализация --- //
  Serializer serializer = Serializer(SerializerParams{...});

  ICPBuilder icp_builder = ICPBuilder(ICPBuilderParams{...});
  ICP icp = icp_builder.build();

  Builder builder = Builder(BuilderParams{...}, icp);
  // --- Инициализация --- //

  // 1) Получаем данные из бэга в виде одометрии и облаков 
  const auto [poses, clouds] = serializer.deserializeBag();

  // 2) Фильтруем полученные данные (делаем разреживание)
  const auto [filtered_poses, filtered_clouds] = builder.filterByPosesProximity(poses, clouds);

  // 3) Уточнем положения облаков через задачу оптимизации
  const auto filtered_poses_optimized = builder.optimizePoses(filtered_poses, filtered_clouds);

  // 4.1) Переводим облака в единую систему координат
  const auto filtered_clouds_tf = builder.transformClouds(filtered_poses_optimized, filtered_clouds);

  // 4.1) Переводим облака в единую систему координат и
  //      объединяем их в единое облако (concatenate = true)
  const auto filtered_cloud_tf = builder.transformClouds(filtered_poses_optimized, filtered_clouds, true)[0];

  // 5.1) Записываем облака в бэг, но здесь мы получим кашу, т.к.
  //      координаты точек каждого из этих облаков посчитаны
  //      в локальной системе координат одометрии [так вызывать функцию сериализации НЕправильно]
  serializer.serializeToMCAP(filtered_clouds);

  // 5.2)  Записываем облака в бэг, здесь получим хорошую карту, т.к.
  //       координаты точек каждого из этих облаков посчитаны
  //       в единой системе координат [так вызывать функцию сериализации правильно]
  serializer.serializeToMCAP(filtered_clouds_tf);  // 1 вариант
  serializer.serializeToMCAP({filtered_cloud_tf}); // 2 вариант
}
```
