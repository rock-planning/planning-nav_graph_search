FLAGS=-O2 -Wno-deprecated
DEFS= -D USE_OPEN_GL -D MACOS

LIBS= -framework OpenGL -framework GLUT -lm -L/usr/local/lib

UNAME= $(shell uname)
ifeq ($(UNAME), Linux)
LIBS= -lGL -lGLU -lglut
DEFS= -D USE_OPEN_GL
endif


all:dstar

dstar: DStarLite.h DStarLite.cpp DStarLiteDraw.cpp
	g++ ${FLAGS} DStarLite.cpp DStarLiteDraw.cpp -o dstar ${LIBS} ${DEFS}

clean:
	rm -f dstar

