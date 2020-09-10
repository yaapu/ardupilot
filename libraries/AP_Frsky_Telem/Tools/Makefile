CC=g++
CFLAGS=-I.
DEPS = mavlite_test.h frsky_sport.h mavlite.h
OBJ = mavlite_test.o frsky_sport.o mavlite.o 

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

mavtest: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -f *.o mavtest