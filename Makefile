BASE:=book
BOOKNAME:=`ls -1 $(BASE)`
SRC:=book/$(BOOKNAME)
IMAGE:=andreacensi/duckuments:devel

all:
	cat README.md

include resources/makefiles/setup.Makefile
