# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.hwlib.Debug:
/Users/s1r/Desktop/computer_graphics_hw2/bin/Debug/libhwlib.a:
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw2/bin/Debug/libhwlib.a


PostBuild.raytrace.Debug:
PostBuild.hwlib.Debug: /Users/s1r/Desktop/computer_graphics_hw2/bin/Debug/raytrace
/Users/s1r/Desktop/computer_graphics_hw2/bin/Debug/raytrace:\
	/Users/s1r/Desktop/computer_graphics_hw2/bin/Debug/libhwlib.a
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw2/bin/Debug/raytrace


PostBuild.hwlib.Release:
/Users/s1r/Desktop/computer_graphics_hw2/bin/Release/libhwlib.a:
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw2/bin/Release/libhwlib.a


PostBuild.raytrace.Release:
PostBuild.hwlib.Release: /Users/s1r/Desktop/computer_graphics_hw2/bin/Release/raytrace
/Users/s1r/Desktop/computer_graphics_hw2/bin/Release/raytrace:\
	/Users/s1r/Desktop/computer_graphics_hw2/bin/Release/libhwlib.a
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw2/bin/Release/raytrace


PostBuild.hwlib.MinSizeRel:
/Users/s1r/Desktop/computer_graphics_hw2/bin/MinSizeRel/libhwlib.a:
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw2/bin/MinSizeRel/libhwlib.a


PostBuild.raytrace.MinSizeRel:
PostBuild.hwlib.MinSizeRel: /Users/s1r/Desktop/computer_graphics_hw2/bin/MinSizeRel/raytrace
/Users/s1r/Desktop/computer_graphics_hw2/bin/MinSizeRel/raytrace:\
	/Users/s1r/Desktop/computer_graphics_hw2/bin/MinSizeRel/libhwlib.a
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw2/bin/MinSizeRel/raytrace


PostBuild.hwlib.RelWithDebInfo:
/Users/s1r/Desktop/computer_graphics_hw2/bin/RelWithDebInfo/libhwlib.a:
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw2/bin/RelWithDebInfo/libhwlib.a


PostBuild.raytrace.RelWithDebInfo:
PostBuild.hwlib.RelWithDebInfo: /Users/s1r/Desktop/computer_graphics_hw2/bin/RelWithDebInfo/raytrace
/Users/s1r/Desktop/computer_graphics_hw2/bin/RelWithDebInfo/raytrace:\
	/Users/s1r/Desktop/computer_graphics_hw2/bin/RelWithDebInfo/libhwlib.a
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw2/bin/RelWithDebInfo/raytrace




# For each target create a dummy ruleso the target does not have to exist
/Users/s1r/Desktop/computer_graphics_hw2/bin/Debug/libhwlib.a:
/Users/s1r/Desktop/computer_graphics_hw2/bin/MinSizeRel/libhwlib.a:
/Users/s1r/Desktop/computer_graphics_hw2/bin/RelWithDebInfo/libhwlib.a:
/Users/s1r/Desktop/computer_graphics_hw2/bin/Release/libhwlib.a:
