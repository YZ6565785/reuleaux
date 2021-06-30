import os
print(os.getcwd())

file = "bl_xarm6_r0.05_reachability.txt"


f = open(os.getcwd()+"/reuleaux/map_creator/maps/"+file, "r")
print("Opening file {}...".format(file))
print("HOLD ON...")
count = 0
total_poses = 0
i = 5
for x in f:
  ri = int(x.split(",")[0])
  if ri > 0:
    count += 1
  total_poses += ri


print("DONE! \nTOTAL available spheres: {}".format(count))
print("TOTAL available poses: {}".format(total_poses))