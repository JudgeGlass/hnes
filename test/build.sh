#!/bin/bash
gcc -g -Wall ../src/*.c *.c -I../include -o test -DTEST -DDISABLE_DEBUG=1