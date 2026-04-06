# Experimental RViz layouts

Configs here are optional. They are not the default for `competition_sim.launch.py`.

- **`rtabmap_nav2_with_fid_cams.rviz`** — extra displays for fiducial debugging. Use with  
  `ros2 launch luna_mapping rtabmap_nav2_sim.launch.py rviz_config:=$(ros2 pkg prefix luna_mapping)/share/luna_mapping/config/experimental/rtabmap_nav2_with_fid_cams.rviz`

The default competition stack uses **`config/competition_sim.rviz`** (passed from `competition_sim.launch.py`).
