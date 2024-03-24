You'll need osm convert!
https://wiki.openstreetmap.org/wiki/Osmconvert#Linux

Step 1: Download the OSM files from https://ad-wiki.informatik.uni-freiburg.de/teaching/EfficientRoutePlanningSS2012

Step 2: Extract into .osm files

Step 3: convert to pbf 
```bash
osmconvert baden-wuerttemberg.osm -o=baden-wuerttemberg.pbf
```