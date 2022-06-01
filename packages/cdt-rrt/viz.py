import json
import matplotlib.pyplot as plt


# Parses JSON
with open("info.json", "r") as read_file:
    data = json.load(read_file)

# Sets the workspace
figure, axes = plt.subplots()
axes.set_xlim([data['x']['min'], data['x']['max']])
axes.set_ylim([data['y']['min'], data['y']['max']])

# Draws the tree
for edge in data["tree"]:
    plt.plot(edge["position"]["x"], edge["position"]["y"], marker='o', color="blue")
    xdisp = [edge["parent"]["x"], edge["position"]["x"]]
    ydisp = [edge["parent"]["y"], edge["position"]["y"]]
    plt.plot(xdisp, ydisp, color="blue")

# Draws the path
for i in range(1, len(data["path"])):
    plt.plot(data["path"][i]["x"], data["path"][i]["y"], marker='o', color="#FFA500")
    xdisp = [data["path"][i-1]["x"], data["path"][i]["x"]]
    ydisp = [data["path"][i-1]["y"], data["path"][i]["y"]]
    plt.plot(xdisp, ydisp, color="#FFA500")

# Plots start and finish
plt.plot(data["start"]["x"], data["start"]["y"], marker='^', color="green", markersize=8)  # #00FF00
plt.plot(data["goal"]["x"], data["goal"]["y"], marker='v', color="red", markersize=8)
draw_circle = plt.Circle((data["goal"]["x"], data["goal"]["y"]), data["radius"], color="r", fill=False)
axes.add_artist(draw_circle)

# Launches the window
plt.grid()
plt.title('Rapidly-Exploring Research Tree')
plt.show()
