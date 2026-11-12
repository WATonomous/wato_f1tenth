std_msgs/Header header
nav_msgs/Path path
TrackWidth[] widths
# Optional measured legal annulus, in header.frame_id, implicitly closed.
# Supply both simple, disjoint, nested loops or leave both empty for width offsets.
nav_msgs/Path[2] legal_boundaries
