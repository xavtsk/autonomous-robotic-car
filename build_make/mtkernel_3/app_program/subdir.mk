################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
# Keep the renamed estimator in its original link position. The legacy path
# below is a sorting key only; compilation uses motion_estimation.c.
APP_ESTIMATOR_SOURCE = ../app_program/subsystem2/motion_estimation.c
APP_ESTIMATOR_SORT_KEY = ../app_program/subsystem2/estimate_motion.c
TEMP_SRCS = $(wildcard ../app_program/*.c) $(subst $(APP_ESTIMATOR_SORT_KEY),$(APP_ESTIMATOR_SOURCE),$(sort $(subst $(APP_ESTIMATOR_SOURCE),$(APP_ESTIMATOR_SORT_KEY),$(wildcard ../app_program/*/*.c))))
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
