
#launcher shell script name
LAUNCHER		:=launcher

#lanch option
LAUNCH_OPTION	:=-g ../opsm-position-tracker-offline.conf

#launch command
LAUNCH_CMD		:=cd $(RELEASE_DIR); ./$(TARGET) $(LAUNCH_OPTION) "$$"@

#shell command interpreter
SHELL_INTRP		:=/bin/bash