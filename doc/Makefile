.PHONY: all

all: ackermann_vehicle_svg

build_dir:
	mkdir -p build

ackermann_vehicle_svg: build_dir
	latex -output-directory build tex/ackermann_vehicle.tex
	dvisvgm --no-fonts build/ackermann_vehicle.dvi svg/ackermann_vehicle.svg


