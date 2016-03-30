v2x-lte: ns-3 extension for vehicular communications over LTE
=============================================================


### An overview
v2x-lte module is a new ns-3 extension containing a CAM implementation on LTE and one example on how to use the new model.

*The code is tested in 3.25 version of ns-3 simulator*


### Build v2v
Copy and paste the v2x-lte package under src/ folder of the ns-3 simulator.

type the command

`./waf configure --enable-examples`

followed by

`./waf`

in the the root directory of the ns-3 simulator. The files built will be copied in the build/ directory.


### Run the v2x-lte example
type the command

`./waf --run src/v2x-lte/examples/v2x-lte-cell`

*The source code of the example can be found in the examples/ directory*


### Acknowledgement

- http://www.ict-itetris.eu/
- https://github.com/alexvoronov/geonetworking
- https://github.com/riebl/vanetza
- https://github.com/bastibl/its-g5-cam
