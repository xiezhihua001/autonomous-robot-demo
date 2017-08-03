all:	
	@make -C lcmtypes
	@make -C bot


groundstation:
	@make -C java
	@make -C lcmtypes


clean:
	@make -C lcmtypes -s clean
	@make -C bot -s clean
	@make -C java -s clean

