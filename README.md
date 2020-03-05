# Cinder-RPLidar
A cinder block talks to RPLidars via USB, with production ready sample.

## Sample

There is a production ready example in the Sample folder, in the xml settings file,
```xml
<lidar>
  <port>com3</port>
	<angle>0</angle>
	<topdown>true</topdown>
	<position>
		<x>215</x>
		<y>5</y>
	</position>
	<min>
		<x>30</x>
		<y>30</y>
	</min>
	<max>
		<x>1503</x>
		<y>821</y>
	</max>
	<slope>0</slope>
	<threshold>1</threshold>
</lidar>
```
Set the lidar's position/angle/port name, min/max for scan area, threshold is the minimum number of points each cluster has to contain, this can effectively remove reflection noises. 

There is also a bug in the SDK I believe, that when closing the app sometimes the Lidar will not stop, so there is a time section in the xml:
```xml
<time>
	<hour>25</hour>
	<minute>1</minute>
</time>
```
After the time set, the app will force the lidar to stop so it doesn't run overnight. Say if the exhibition ends at 6pm, you can set the time to be 
```xml
<time>
	<hour>18</hour>
	<minute>1</minute>
</time>
```
Then lidar will stop at 6:01pm.

Also to deal with possible floor reflections, filter dot can be added with x/y/r(adius) parameters, anything within that circle range will be considered noise and removed before sending for clustering: 
```xml
<filter>
	<dot>
		<x>1400</x>
		<y>1350</y>
		<r>50</r>
	</dot>
</filter>
```
