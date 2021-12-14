.PHONY: uf2
uf2:
	rm -rf build
	mkdir build
	cmake -B build
	$(MAKE) -C build