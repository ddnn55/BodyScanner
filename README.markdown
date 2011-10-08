BodyScanner
===========

BodyScanner is an easy to use, open source Body Scanning solution running on top of OpenNI (openni.org) and the Point Cloud Library (pointclouds.org).

BodyScanner is in it's very first moments of life! It is not yet ready for general consumption.


General Build
-------------

To build, type the following in this directory:

mkdir build
cd build
cmake ..
make

This will build all the subprojects.


Data
----

Expand the mark1_pcd.tar.bz2 archive into the BodyScanner/recordings/mark1_pcd folder.
This folder is ignored by git, so it will not be committed.

When our programs are run, they will be called from their directories inside BodyScanner/build/subprojectXYZ/
So when loading a PCD file, go up two directories, like this: "../../recordings/mark1_pcd/1316652689.744386960.pcd"


Surface Reconstructor
---------------------

To build a triangle mesh from a scan,
type the following in the build/surface-reconstructor directory

make
./surface-reconstructor -s ../../recordings/mark1_pcd/1316652689.744386960.pcd # read and smooth point cloud
./surface-reconstructor -r # reconstruct the surface
./vtk2obj.py scan.vtk scan.obj # convert into a format that meshlab recognizes (and re-orient faces)

To view the scan.obj that was created, install meshlab:

sudo apt-get install meshlab

And run it:

meshlab scan.obj &


Exporting to FBX
----------------

Grab the SDK and install it with these commands:

wget http://luffel.org/2011/body-scanner/fbx20122_fbxsdk_linux.tar.gz
tar -xzvf fbx20122_fbxsdk_linux.tar.gz
chmod u+x fbx20122_fbxfilesdk_linux
sudo ./fbx20122_fbxfilesdk_linux /usr


Full export
-----------

Here's how to build a fully rigged mesh from the skeleton and point cloud data.
Not that this totally works.

DATA=recordings/manohar
OUTPUT=records/manohar_files
PCD=$DATA/manohar_00001.pcd
YAML=$DATA/manohar_skeleton_00001.yaml
WEI=$OUTPUT/00001.w
OBJ=$OUTPUT/00001.obj
FBX=$OUTPUT/00001.fbx
mkdir $OUTPUT

segmentation/build/sharpSegmentation $PCD $YAML $WEI
surface-reconstructor/build/surface-reconstructor $PCD $OBJ
python MakeRig.py $YAML $OBJ $WEI $FBX


