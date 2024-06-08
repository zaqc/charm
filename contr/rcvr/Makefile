CC = iverilog
FLAGS = -Wall -Winfloop -g2005-sv -DTESTMODE=1 -DTARGET_NAME="\""$(TARGET)"\""

TARGET = rcvr

SRC = $(TARGET)_tb.v $(TARGET).v

$(TARGET) : $(SRC) Makefile
	$(CC) $(FLAGS) -o $(TARGET) $(SRC)
	vvp $(TARGET)
	gtkwave --rcvar 'fontname_waves Ubuntu Mono 11' dumpfile_$(TARGET).vcd cfg_$(TARGET).gtkw
	rm -f $(TARGET)
#--rcvar 'fontname_signals Ubuntu Mono 14' --rcvar 'fontname_waves Ubuntu Mono 12' 
wave:
	gtkwave --rcvar 'fontname_signals Monospace 18' dumpfile_$(TARGET).vcd cfg_$(TARGET).gtkw
	
edit:
	gedit -s $(SRC) Makefile desc.txt &
	
clean:
	rm -f $(TARGET)

