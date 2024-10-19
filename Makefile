WORKDIR ?= $(shell pwd)

.PHONY: build
build:
	docker run -it -v ${WORKDIR}:/work -w /work -e GRADLE_USER_HOME=/work/.gradle-home wpilib/roborio-cross-ubuntu:2024-22.04 sh -c "./gradlew build"

.PHONY: deploy
deploy:
	docker run -it -v ${WORKDIR}:/work -w /work -e GRADLE_USER_HOME=/work/.gradle-home wpilib/roborio-cross-ubuntu:2024-22.04 sh -c "./gradlew deploy"

.PHONY: clean
clean:
	docker run -it -v ${WORKDIR}:/work -w /work -e GRADLE_USER_HOME=/work/.gradle-home wpilib/roborio-cross-ubuntu:2024-22.04 sh -c "rm -Rf build"

.PHONY: deep-clean
deep-clean: clean
	docker run -it -v ${WORKDIR}:/work -w /work -e GRADLE_USER_HOME=/work/.gradle-home wpilib/roborio-cross-ubuntu:2024-22.04 sh -c "rm -Rf .gradle && rm -Rf .gradle-home"
	git clean -fdx