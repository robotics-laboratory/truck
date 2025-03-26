import open3d as o3d


def pcd_to_ply(pcd_file_path, ply_file_path):
    # Чтение файла PCD
    pcd = o3d.io.read_point_cloud(pcd_file_path)

    # Проверка, был ли успешно загружен PCD файл
    if not pcd.is_empty():
        # Сохранение в формат PLY
        o3d.io.write_point_cloud(ply_file_path, pcd)
        print(f"Файл успешно конвертирован в {ply_file_path}")
    else:
        print("Не удалось загрузить PCD файл.")


pcd_paths = [
    "1.pcd",
    "2.pcd",
    "3.pcd",
    "4.pcd",
    "5.pcd",
    "6.pcd",
    "7.pcd",
    "people_sprava_10m.pcd",
    "9.pcd",
    "10.pcd",
]

input_dir = "output_pcd/"
output_dir = "output_ply/"

for pcd_path in pcd_paths:
    ply_path = f"{pcd_path[:-4]}.ply"
    # Конвертирование PCD в PLY
    pcd_to_ply(input_dir + pcd_path, output_dir + ply_path)
