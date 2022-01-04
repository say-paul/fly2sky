.PHONY: uf2
uf2:
	rm -rf build
	mkdir build
	cmake -B build
	$(MAKE) -C build

.PHONY: camera-setup
camera-setup:
	scp -r flight_computer/* pi@raspberrypi.local:/home/pi/workspace/
	ssh pi@raspberrypi.local python3 /home/pi/workspace/camera.py

.PHONY: camera-fetch
camera-fetch:
	rm -rf Piworkspace/
	mkdir Piworkspace
	scp -r pi@raspberrypi.local:/home/pi/workspace/*  Piworkspace/

.PHONY: camera
camera: camera-setup camera-fetch
