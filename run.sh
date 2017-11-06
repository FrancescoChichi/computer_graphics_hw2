./bin/raytrace -r 360 -s 3 -f -o out/subdiv_cube_base.png in/subdiv_cube_pointlight/subdiv_cube_pointlight.obj
./bin/raytrace -r 360 -s 3 -S -f -o out/subdiv_cube_faceted.png in/subdiv_cube_pointlight/subdiv_cube_pointlight.obj
./bin/raytrace -r 360 -s 3 -S -o out/subdiv_cube_smooth.png in/subdiv_cube_pointlight/subdiv_cube_pointlight.obj

./bin/raytrace -r 360 -s 3 -f -o out/subdiv_suzanne_base.png in/subdiv_suzanne_pointlight/subdiv_suzanne_pointlight.obj
./bin/raytrace -r 360 -s 3 -S -f -o out/subdiv_suzanne_faceted.png in/subdiv_suzanne_pointlight/subdiv_suzanne_pointlight.obj
./bin/raytrace -r 360 -s 3 -S -o out/subdiv_suzanne_smooth.png in/subdiv_suzanne_pointlight/subdiv_suzanne_pointlight.obj

./bin/raytrace -r 360 -s 3 -T -o out/normdisp_smooth.png in/normdisp_pointlight/normdisp_pointlight.obj

./bin/raytrace -r 360 -s 3 -H 65536 -o out/simple_hair.png in/simple_pointlight/simple_pointlight.obj
./bin/raytrace -r 360 -s 3 -S -H 65536 -o out/subdiv_suzanne_hair.png in/subdiv_suzanne_pointlight/subdiv_suzanne_pointlight.obj

./bin/raytrace -r 360 -s 3 -C 65536 -o out/simple_curve.png in/simple_pointlight/simple_pointlight.obj
./bin/raytrace -r 360 -s 3 -S -C 65536 -o out/subdiv_suzanne_curve.png in/subdiv_suzanne_pointlight/subdiv_suzanne_pointlight.obj
