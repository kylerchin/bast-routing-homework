You'll need osm convert!
https://wiki.openstreetmap.org/wiki/Osmconvert#Linux

Step 1: Download the OSM files from https://ad-wiki.informatik.uni-freiburg.de/teaching/EfficientRoutePlanningSS2012

Step 2: Extract into .osm files

Step 3: convert to pbf 
```bash
osmconvert baden-wuerttemberg.osm -o=bast-baden-wuerttemberg.pbf
```

----------

It's probably better to run the examples with more recent data.

https://download.geofabrik.de/europe/germany/baden-wuerttemberg-latest.osm.pbf
https://download.geofabrik.de/europe/germany/saarland-latest.osm.pbf

Testing can be done via 
```bash
cargo test --release -- --nocapture
```