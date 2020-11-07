options=env -u DTSERVER -u DTSHELL_COMMANDS -u TWINE_USERNAME -u TWINE_PASSWORD  -u TWINE_REPOSITORY_URL -u PIP_INDEX_URL

# -u AIDO_REGISTRY
all: \
	rec-challenges-info.gif \
	rec-submit.gif \
	rec-evaluate.gif \
	rec-dts-version.gif \
	rec-dts-command-version.gif \
	rec-dts-update.gif

clean:
	rm -f *.gif *.cast

%.cast: %.cmd
	$(options) asciinema rec $@ --overwrite -c "$(shell cat $<)"

%.gif: %.cast
	docker run --rm -v $(PWD):/data asciinema/asciicast2gif -s 1 $< $@

# -w 120 -h 30 