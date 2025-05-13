import json
from collections import defaultdict
import numpy as np

def load_marker_data(path):
    with open(path, "r") as file:
        return {
            frame["frame_num"]: {c["marker_id"] for c in frame["centers"]}
            for frame in json.load(file)
        }

def compare_marker_files(file1_path, file2_path):
    data1 = load_marker_data(file1_path)
    data2 = load_marker_data(file2_path)

    stats = np.zeros((3,3), dtype=('int32'))
    all_frames = sorted(set(data1.keys()) | set(data2.keys()))

    file1_better = []
    file2_better = []

    file1_invalid = []
    file2_invalid = []

    for frame in all_frames:
        ids1 = data1.get(frame, set())
        ids2 = data2.get(frame, set())

        if not ids1.issubset({0, 1}):
            file1_invalid.append(frame)

        if not ids2.issubset({0, 1}):
            file2_invalid.append(frame)

        ids1 = ids1.intersection({0, 1})
        ids2 = ids2.intersection({0, 1})

        stats[len(ids1)][len(ids2)] += 1

        if (len(ids2) == 1):
            print(frame)
        if (len(ids1) > len(ids2)):
            file1_better.append(frame)
        if (len(ids2) > len(ids1)):
            file2_better.append(frame)
        # if not ids1 and not ids2:
        #     stats["both_empty"] += 1
        # elif ids1 == {0, 1} and ids2 == {0, 1}:
        #     stats["both_full"] += 1
        # else:
        #     if ids1 == {0, 1} and ids2 in [{0}, {1}]:
        #         stats["file1_both_file2_one"] += 1
        #     if ids2 == {0, 1} and ids1 in [{0}, {1}]:
        #         stats["file2_both_file1_one"] += 1
        #     if not ids2 and ids1 in [{0}, {1}]:
        #         stats["file2_empty_file1_one"] += 1
        #     if not ids1 and ids2 in [{0}, {1}]:
        #         stats["file1_empty_file2_one"] += 1

    return stats, file1_better, file2_better, file1_invalid, file2_invalid

if __name__ == "__main__":
    file1 = "./odometry_scripts/data/run-06/markers.json"
    file2 = "./odometry_scripts/data/run-06/markers.json"

    stats, file1_better, file2_better, file1_invalid, file2_invalid = compare_marker_files(file1, file2)

    print("\nStat:")
    print(stats, file1_better, file2_better, file1_invalid, file2_invalid, sep='\n\n')