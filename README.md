# Exjobb
Exjobb

The goal of this project is to detect, track and determine the trajectory of a moving
object. To achieve this, the software implementation will include algorithms for motion
detection, edge detection, clustering and RANSAC for keeping track of the interesting
object by eliminating outliers. A system architecture will appear as follows:
The frames are read from the bag file, then background subtraction is applied to detect
motion. If there is movement of objects located in a frame, then clustering is applied to
assign points into clusters. Different clusters are classified as different objects. Then
RANSAC is used to keep track on the cluster that forms a parabolic movement. The
model is also used to predict the continuation of the trajectory by extending the curve.
Finally, the curve is checked whether it intersects with the area of the box or not to
decide if the throw was successful or not.

![image](https://github.com/samiG99/Exjobb/assets/58847870/e9b2265f-f175-4d50-8e71-1eb5bfe24af2)
