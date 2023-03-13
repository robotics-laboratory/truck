# CollisonChecker
Tiny library to evaluate distance to nearest obstacle in a 2D space (occupancy grid representation):
- pre-compute distance to nearest obstacle for each cell in the map O(n) complexity
- request for truck pose O(1) complexity

## Details
Truck shape is decomposed as a set of intersecting circles. The distance to nearest obstacle is computed as a minimum distance to nearest circle.