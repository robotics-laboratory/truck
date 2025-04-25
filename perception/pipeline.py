import os
import shutil

from config import Config
from frames_from_mp4 import get_frames_from_mp4_main
from pcd_utils.mcap_to_pcd import mcap_to_pcd_main

run_name = Config.run_name

get_frames_from_mp4_main()

mcap_to_pcd_main()


output_dir = "output_zip"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
# Создание пути к архиву
archive_path = os.path.join(output_dir, run_name)
# Архивируем папку без сжатия
shutil.make_archive(archive_path, "zip", root_dir=run_name)
# Удаляем исходную папку
shutil.rmtree(run_name)
print(f"Папка {run_name} архивирована в {output_dir}/{run_name}.zip и удалена")
